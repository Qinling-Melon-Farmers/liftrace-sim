#!/usr/bin/env python3
"""Small report for current-scene screening; plotting is optional."""
import argparse,json,statistics
from pathlib import Path

def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--screen',type=Path,required=True);ap.add_argument('--scenes',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);ap.add_argument('--plot',action='store_true');a=ap.parse_args()
    data=json.loads(a.screen.read_text());scenes=json.loads(a.scenes.read_text());a.output.mkdir(parents=True,exist_ok=True)
    rows=data['rows'];summary=[]
    for strategy in ('high','low_coverage'):
        selected=[r for r in rows if r['strategy']==strategy];valid=[r for r in selected if r['status']=='GEOMETRY_FEASIBLE']
        entry=dict(strategy=strategy,cases=len(selected),geometry_feasible=len(valid))
        for mode in ('center','refine'):
            values=[r['geometry_all_three'][mode]['time_s'] for r in valid if r['geometry_all_three'][mode] is not None]
            entry[mode+'_all_three_count']=len(values);entry[mode+'_time_median_s']=statistics.median(values) if values else None
        summary.append(entry)
    (a.output/'summary.json').write_text(json.dumps(dict(scope=data['scope'],rows=summary),indent=2))
    lines=['# 当前高位与低位路线几何预筛','',
           '范围：冻结31–40布局，准确静态地图假设、树冠近似、固定姿态、恒速与转弯惩罚。不是ROS/Gazebo或检测器，也不是整场Gate。',
           '55cm为已膨胀鲁棒设计包络，默认额外膨胀0；导航用全高柱，视线遮挡仍用物体高度。目标坐标只在路线生成后用于几何可见性评估。', '',
           '本次导入来源均为高位实跑。每个seed两行复用同一来源Gate，low_coverage行并不表示该低位路线被实际飞过。', '',
           '| seed | 路线 | 几何可行 | 航程m | 全线名义s | 累计三类中心可见s | 累计三类外圈几何可见s | 同源高位实跑Gate |',
           '|---|---|---|---:|---:|---:|---:|---|']
    def fmt(v):return '—' if v is None else f'{v:.2f}'
    for r in rows:
        visible=r.get('geometry_all_three',{});times=[visible.get(m,{}).get('time_s') if visible.get(m) else None for m in ('center','refine')]
        lines.append(f"| {r['seed']} | {r['strategy']} | {r['status']} | {fmt(r.get('route_distance_m'))} | {fmt(r.get('nominal_full_search_s'))} | {fmt(times[0])} | {fmt(times[1])} | {r['actual_gate']} |")
    lines+=['','## 如何解释','',
            '两种可见性口径都要求配置的连续可见驻留，按固定10Hz右端采样；转向计时但不累计观测。这只是几何代理，不是high-view Catalog的质量/置信度/类别投票。中心与外圈应分别比较，不能选择更好看的口径宣布任务成功。',
            '表内三类始终为bridge/panzer/red_cross，不是任意最先看到的三个靶。即使三类全部几何可见，也没有模拟低位复访、投递、走廊或降落；整场时间与检测概率输出null。',
            '全高导航柱与真实高度LOS分开，避免高位直穿树投影；二维准确地图A*仍是乐观先验，不能定位实际进度停滞或控制超调。树冠高度1.8m及排除半径AABB是显式近似，可做敏感性评估，不是精确网格重建。',
            '两路线具有不同的代表速度0.8/0.6m/s；本表是当前配置筛选，不是高度的单变量因果比较。要隔离路线收益，应另用相同速度配置。', '',
            '汇总中各时间中位数只针对该方法达到三类的子集；高位7例与低位10例不是同一分母，不能相除宣称整批节时。名义路线几何与实际避障轨迹/姿态不同，实际PASS但名义路线几何不齐也可能发生。', '',
            '仅有单seed B100的概率表没有2.6/1.4m精确格点；本入口不调用其legacy概率回退，不把几何可见性包装为P_interrupt。']
    if a.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle,Patch
        n=len(scenes['cases']);fig,axes=plt.subplots((n+4)//5,5,figsize=(20,4.5*((n+4)//5)),squeeze=False)
        for ax,case in zip(axes.flat,scenes['cases']):
            for w in case['walls']:ax.add_patch(Rectangle((w['x']-w['size_x']/2,w['y']-w['size_y']/2),w['size_x'],w['size_y'],color='#a9adaf'))
            for t in case['trees']:ax.add_patch(Rectangle((t['x']-t['radius'],t['y']-t['radius']),2*t['radius'],2*t['radius'],color='#7daf86',alpha=.6))
            for strategy,color in [('low_coverage','#b8bcc1'),('high','#307caa')]:
                row=next(r for r in rows if r['run']==case['run'] and r['strategy']==strategy)
                if row.get('route'):
                    ax.plot([p[0] for p in row['route']],[p[1] for p in row['route']],color=color,lw=1.0 if strategy=='high' else .65,label=strategy)
            for t in case['targets']:
                required=t['class_name'] in data['profile']['required_classes']
                ax.scatter(t['x'],t['y'],marker='s',s=18,color='#b14436' if required else '#777')
            ax.set(xlim=(-5.1,5.1),ylim=(-.8,9.5),title=f"Seed {case['seed']} | observed {case['actual_gate']}",xlabel='World X (m)',ylabel='World Y (m)');ax.set_aspect('equal');ax.grid(alpha=.15)
        for ax in list(axes.flat)[n:]:ax.set_visible(False)
        fig.suptitle('Known-map geometry only | blue: high route, grey: low coverage | raw obstacles shown, 0.55m guard applied once',fontsize=12)
        fig.tight_layout(rect=(0,0,1,.97));fig.savefig(a.output/'routes.png',dpi=160);plt.close(fig)
        lines+=['','![Geometry routes](routes.png)']
    (a.output/'REPORT.md').write_text('\n'.join(lines)+'\n');print(json.dumps(summary,indent=2))

if __name__=='__main__':main()
