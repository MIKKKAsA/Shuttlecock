# -*- coding: utf-8 -*-
import math

NET_X, NET_H = 6.7, 1.5
G = 9.8
# ===== 身体位置 =====
HIP_X, HIP_Y = -0.60, 0.85        # 髋：控制整体位移
THIGH_L = 0.25                    # 大腿固定长度
LEG_ANG = 45                      # 击球时大腿相对髋垂线角度（°），0=竖直；准备时为 0
# ===== 膝盖弯曲（驱动击球瞬间）=====
BETA_SET = 90                     # 击球时膝盖角：0=折叠垂直，90=伸直直线（不可超过）；准备时为 0
# ===== 腿/板尺寸 =====
L1, L2 = 0.40, 0.24
BOARD_T, BR = 0.006, 0.055
T = 0.30
E_REST, MU = 0.4, 0.15
DRAG = 0.6
BOARD_S = 0.20                     # 球沿脚面距脚踝距离（0=踝→0.24=脚尖）

def knee():
    a=math.radians(LEG_ANG)
    return (HIP_X+THIGH_L*math.sin(a), HIP_Y-THIGH_L*math.cos(a))

def rot(v,a):
    c,s=math.cos(a),math.sin(a)
    return (c*v[0]-s*v[1], s*v[0]+c*v[1])

def phi_abs(theta,beta):           # 小腿绝对方向（相对竖直）= 大腿角 - 90 + 膝盖角
    return math.radians(theta-90+beta)

def pose(phi, ang, kx, ky):
    d1=(math.sin(phi),-math.cos(phi))
    d2=rot(d1, math.radians(90-ang))
    nrm=rot(d2, math.pi/2)
    ank=(kx+L1*d1[0], ky+L1*d1[1])
    toe=(ank[0]+L2*d2[0], ank[1]+L2*d2[1])
    return d2,nrm,ank,toe

# 球的起始位置：贴在击球姿态脚面表面
def ball_start(ang,beta_set,theta_set):
    phi=phi_abs(theta_set,beta_set)
    kx=HIP_X+THIGH_L*math.sin(math.radians(theta_set))
    ky=HIP_Y-THIGH_L*math.cos(math.radians(theta_set))
    d2,nrm,ank,toe=pose(phi,ang,kx,ky)
    p=(ank[0]+BOARD_S*d2[0],ank[1]+BOARD_S*d2[1])
    return (p[0]+nrm[0]*BR,p[1]+nrm[1]*BR)

def closest(p,a,b):
    dx,dy=b[0]-a[0],b[1]-a[1]; l2=dx*dx+dy*dy
    u=max(0,min(1,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/l2)) if l2 else 0
    return (a[0]+u*dx,a[1]+u*dy)

def predict(ang,power,bx,by,kx,ky,beta_set,theta_set):
    Tsw=T*(100.0/power)
    N=1500
    for i in range(N+1):
        u=i/N
        theta=theta_set*u*u
        beta=beta_set*u*u
        # 大腿、小腿联动：总角速度 = (大腿角+膝盖角) 随时间变化
        w=math.radians(theta_set+beta_set)/Tsw*2*u
        phi=phi_abs(theta,beta)
        kx=HIP_X+THIGH_L*math.sin(math.radians(theta))
        ky=HIP_Y-THIGH_L*math.cos(math.radians(theta))
        d2,nrm,ank,toe=pose(phi,ang,kx,ky)
        c=closest((bx,by),ank,toe)
        d=math.hypot(bx-c[0],by-c[1])
        if d<=BR+BOARD_T:
            nx=(bx-c[0])/max(d,1e-9);ny=(by-c[1])/max(d,1e-9)
            vs=(-w*(c[1]-ky), w*(c[0]-kx))
            if vs[0]*nx+vs[1]*ny>0:
                vn0=(-vs[0])*nx+(-vs[1])*ny
                vt0=(-vs[0])*d2[0]+(-vs[1])*d2[1]
                vx=vs[0]+(-E_REST*vn0)*nx+(1-MU)*vt0*d2[0]
                vy=vs[1]+(-E_REST*vn0)*ny+(1-MU)*vt0*d2[1]
                return vx,vy,phi,beta
    return None

def simulate(ang=90,power=100,board_s=BOARD_S,drag=DRAG,beta_set=None,
             theta_set=None):
    global BOARD_S
    if beta_set is None:beta_set=BETA_SET
    if theta_set is None:theta_set=LEG_ANG
    BOARD_S=board_s
    bx,by=ball_start(ang,beta_set,theta_set)
    pr=predict(ang,power,bx,by,HIP_X,HIP_Y-THIGH_L,beta_set,theta_set)
    if pr is None: return "未踢中"
    vx,vy,phi,beta=pr
    x,y=bx,by
    h=1/2400;netclear=None;netv=None;nethit=False;apex=y;ft=0
    while True:
        vx+=-drag*vx*h;vy+=(-G-drag*vy)*h
        px,py=x,y;x+=vx*h;y+=vy*h;ft+=h;apex=max(apex,y)
        if netclear is None and px<NET_X and x>=NET_X:
            uu=(NET_X-px)/(x-px);yat=py+uu*(y-py)
            netclear=yat-NET_H
            netv=math.hypot(vx,vy)
            if yat-BR<=NET_H:nethit=True
        if nethit:x=NET_X;vx=0;vy=-0.6
        if y<=BR:landed=x;break
    verdict='下网' if nethit else('出界' if landed>13.4 else('未过网' if landed<=NET_X else '界内'))
    pv=math.hypot(pr[0],pr[1]);pa=math.degrees(math.atan2(pr[1],pr[0]))
    return dict(pred_ang=pa,pred_spd=pv,netclear=netclear,netv=netv,apex=apex,
                landed=landed,land_from_net=landed-NET_X,flight_t=ft,
                verdict=verdict,hit_beta=beta)

r=simulate()
if isinstance(r,str):print(f"默认: {r}")
else:
 print(f"默认(beta{BETA_SET}): {r['verdict']} | 击球时膝盖角{r['hit_beta']:.1f} 出射{r['pred_ang']:.1f}° 速{r['pred_spd']:.1f} "
      f"网{r['netclear']:+.2f} 距网{r['land_from_net']:.2f} apex{r['apex']:.2f}")
print('--- 摆腿目标膝盖角 BETA_SET ---')
for bs in [60,70,80,85,90]:
    r=simulate(beta_set=bs)
    if isinstance(r,str):print(f"beta{bs}: {r}");continue
    nc='  —  ' if r['netclear'] is None else f"{r['netclear']:+.2f}"
    dn='  —  ' if r['land_from_net'] is None else f"{r['land_from_net']:.2f}"
    print(f"set{bs}: {r['verdict']} 击球β{r['hit_beta']:.1f} 出射{r['pred_ang']:.1f} 速{r['pred_spd']:.1f} 网{nc} 距网{dn}")
print('--- 整体位移 HIP_X ---')
for hx in [-1.2,-0.9,-0.6,-0.3,0.0]:
    HIP_X=hx; r=simulate()
    if isinstance(r,str):print(f"hip{hx}: {r}");continue
    print(f"hip{hx:+.2f}: {r['verdict']} 网{r['netclear']:+.2f} 距网{r['land_from_net']:.2f}")
HIP_X=-0.60
def score_of(r):
    if r['verdict']!='界内':return None
    d=r['land_from_net']; t=d/6.7
    landX=abs(d-3.35)/3.35
    v=max(0,min(1,(r['netv']-3)/10))
    vMatch=t*v+(1-t)*(1-v)
    netH=max(0,min(1,1-(r['netclear'] or 0)/1.0))
    return round(100*(0.45*landX+0.35*vMatch+0.20*netH))

print('--- 网格扫描：最高能拿多少分 ---')
best=[]
for pw in [60,80,100,120,150,180,220,260,300]:
  for bs in [0,0.04,0.08,0.12,0.16,0.20,0.24]:
    for ang in [0,20,40,60,80,90]:
      for th in [0,20,35,45,55,60]:
        r=simulate(ang=ang,power=pw,board_s=bs,theta_set=th,beta_set=90)
        if isinstance(r,str):continue
        sc=score_of(r)
        if sc is not None:
          best.append((sc,pw,bs,ang,th,r['land_from_net'],r['netv'],r['netclear']))
best.sort(reverse=True)
print('得分最高的 10 组：')
for row in best[:10]:
  print(f"  评分{row[0]:3d} 力度{row[1]} 球位{row[2]:.2f} 脚面角{row[3]} 大腿角{row[4]} 距网{row[5]:.2f} 过网速{row[6]:.1f} 网余{row[7]:+.2f}")
front=[x for x in best if x[5]<1.5]; back=[x for x in best if x[5]>5.2]
if front:print('前场最高:',front[0])
if back:print('后场最高:',back[0])

print('--- 局部细化（后场最优附近）---')
fine=[]
for pw in range(180,271,10):
  for bs in [0.0,0.02,0.04,0.06,0.08]:
    for ang in [84,88,90]:
      for th in [12,16,20,24,28]:
        r=simulate(ang=ang,power=pw,board_s=bs,theta_set=th,beta_set=90)
        if isinstance(r,str):continue
        sc=score_of(r)
        if sc is not None:fine.append((sc,pw,bs,ang,th,r['land_from_net'],r['netv'],r['netclear']))
fine.sort(reverse=True)
for row in fine[:5]:
  print(f"  评分{row[0]:3d} 力度{row[1]} 球位{row[2]:.2f} 脚面{row[3]} 大腿{row[4]} 距网{row[5]:.2f} 过网速{row[6]:.1f} 网余{row[7]:+.2f}")
