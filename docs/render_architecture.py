"""Architecture snapshot grounded in the October 15 morning report."""
from pathlib import Path
import argparse
import math
from PIL import Image, ImageDraw, ImageFont

p = argparse.ArgumentParser()
p.add_argument('--font', required=True)
a = p.parse_args()
im = Image.new('RGB', (2400, 2190), '#f5f8fc')
d = ImageDraw.Draw(im)
blue, orange, ink, gray = '#2263aa', '#bc7029', '#192d45', '#53657a'
def txt(x,y,s,n=29,color=ink):
    d.text((x,y),s,font=ImageFont.truetype(a.font,n),fill=color)
def line(points,planned=False):
    c=orange if planned else blue
    for (x,y),(u,v) in zip(points,points[1:]):
        dist=math.hypot(u-x,v-y)
        if not planned:
            d.line((x,y,u,v),fill=c,width=5)
        else:
            for start in range(0,int(dist),24):
                end=min(start+13,dist)
                d.line((x+(u-x)*start/dist,y+(v-y)*start/dist,x+(u-x)*end/dist,y+(v-y)*end/dist),fill=c,width=5)
def arrow(points,planned=False):
    line(points,planned)
    x,y=points[-1];u,v=points[-2];t=math.atan2(y-v,x-u)
    d.polygon([(x,y),(x-19*math.cos(t-.45),y-19*math.sin(t-.45)),(x-19*math.cos(t+.45),y-19*math.sin(t+.45))],fill=orange if planned else blue)
def panel(x,y,w,h,title):
    d.rounded_rectangle((x,y,x+w,y+h),radius=24,fill='#eaf1fb',outline='#c7d8ef',width=3)
    txt(x+24,y+15,title,34,blue)
def box(x,y,w,h,title,lines=(),planned=False):
    d.rounded_rectangle((x,y,x+w,y+h),radius=15,fill='#fffaf3' if planned else 'white',outline='#dec49f' if planned else '#8baed8',width=3)
    txt(x+18,y+12,title,32,orange if planned else blue)
    for i,s in enumerate(lines):txt(x+18,y+61+37*i,s,27,gray)

txt(60,30,'robot-1｜コードと実機ログを照合したアーキテクチャ',48)
txt(60,100,'2026-10-17 19時整理 · main: 2e04b7e ／ 開発ブランチ: eb74cb1（主な調査対象）',28,gray)
line([(70,168),(145,168)]);txt(165,148,'現行コードの経路・部品',27)
line([(730,168),(805,168)],True);txt(825,148,'手動接続・要設定・未統合',27,orange)
txt(1520,148,'実機検証の成否は下部に分けて記載',26,gray)

panel(60,230,2280,300,'現在の入力・制御｜Mac + Colima / Docker / ROS 2')
box(100,310,480,155,'vision.py / YOLO',
    ['USBカメラ または MJPEG','HTTP JSON :8765で検出を配信'])
box(675,310,730,155,'webcam_bridge → vision_turn_controller',
    ['検出選択・画素→odom・目標管理・制御が同居','map_frame_adapterで目標と姿勢をmapへ統一'])
box(1500,310,800,155,'Navigationの速度出力',
    ['現行既定：/cmd_vel_unstamped','設定だけでは出力先の排他切替を保証しない'])
arrow([(580,390),(675,390)])
arrow([(1405,390),(1500,390)])

panel(60,610,2280,310,'現在の出力｜Create 3直結と、手動追加の出力切替が共存')
box(100,695,640,160,'既存：Create 3用relay',
    ['Twist → TwistStamped /cmd_vel','Create 3 / Gazeboへ直接送信'])
box(865,695,700,160,'cmd_vel_output_adapter.py',
    ['gazebo / dry_run / roombaを選択','追加起動が必要。入力・出力topicに注意'])
box(1690,695,610,160,'roomba_bridge_tcp.py',
    ['専用 /roomba/cmd_vel_unstamped','STOP確認・OI Safe・ARM・速度制限'])
arrow([(1850,465),(1850,570),(420,570),(420,695)])
arrow([(2150,465),(2150,550),(1215,550),(1215,695)],True)
arrow([(1565,775),(1690,775)])

panel(60,1000,2280,335,'実機への通信経路｜ROS側TCP → Pico W → UART → Roomba 680')
box(100,1090,770,165,'roomba_tcp_transport.py → TCP',
    ['READY / STOP ACK / OI Safe / ARM世代','世代付きDrive。世代は指令ごとの時刻ではない'])
box(965,1090,715,165,'pico/pico_relay.py',
    ['STOPPED / ARMED / LATCHED_STOP','500 ms監視・世代照合・固定フレーム検査'])
box(1775,1090,525,165,'Roomba 680 / OI',
    ['UART：右輪→左輪 mm/s','実機姿勢のROS供給は未統合'])
arrow([(2000,855),(2000,960),(485,960),(485,1090)])
arrow([(870,1170),(965,1170)])
arrow([(1680,1170),(1775,1170)])
txt(100,1280,'配布注意：Pico上のmain.pyが実行版。リポジトリのpico/main.pyはプローブ。秘密情報と版を分離して管理。',27,gray)

panel(60,1400,2280,285,'整理後の標準経路｜設計案・今回未適用')
box(100,1485,565,135,'共通Navigation',
    ['/nav/cmd_vel_unstampedへ一度だけ出力'],True)
box(760,1485,665,135,'出力切替：1モードのみ',
    ['gazebo / dry_run / roombaの起動設定'],True)
box(1520,1485,780,135,'通信と安全状態は接続先に閉じ込める',
    ['実機姿勢と目標はmapで統一。計算coreはROS非依存'],True)
arrow([(665,1550),(760,1550)],True)
arrow([(1425,1550),(1520,1550)],True)

panel(60,1750,2280,325,'現在地と優先事項｜コード実装と実機合格を分ける')
rows=[
 ('確認済みの報告','Create 3：YOLO接近・停止成功。Pico：115200 bps応答、TCP STOP、watchdog 503/504 ms'),
 ('最優先の確認','リレーコード既定19200と実機成功115200の不一致を解消し、配布版・設定を記録'),
 ('停止機構','最新コードにラッチ・ARMとテストあり。実機での旧指令拒否・物理停止維持は未合格'),
 ('次の整理','出力を一本化 → 計算coreを抽出 → 実機姿勢を接続。450枚の再学習は後段'),
]
for i,(title,detail) in enumerate(rows):
 yy=1835+i*52
 txt(100,yy,title,29,blue if i==0 else orange)
 txt(440,yy+2,detail,25,gray)
txt(70,2110,'今回：文書・図のみ更新。安全ロック・baud・走行コード・Picoファームウェアは変更していない。',27,gray)
txt(70,2150,'検証：停止・通信関連56テスト成功。全体はcv2不在で3モジュールimport不可。実機再試験は未実施。',25,gray)
out=Path(__file__).with_name('architecture_2026-10-17.png')
im.save(out)
print(out.resolve())
