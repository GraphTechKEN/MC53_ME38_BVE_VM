# MC53-ME38_BVE_VM
本開発品は主に国鉄型車両向けの速度計や電圧計のメーターとマスコン、ブレーキ弁などの外部機器をBVEと連動させるものです。

> [!TIP]
>- BVE5.8と6の両対応
>- Arduino Microを使用し、HIDキーボード機能を利用したキー入力と、シリアルポートを介した機器連動を同時に実現します。
>- BVEだけではなく、トレインシミュレータなどとも連携が可能(メーター類は不動)
>- 部品の配線やポテンショの追加改造のみで実装可能
>- 付属の調整ソフトで、ブレーキ弁角度などの細かい設定も可能

## 製作方法
1. 基板は秋月電子製片面紙エポキシ・ユニバーサル基板　Ａの小タイプ(AE-5)またはサンハヤト製(ICB-97 1.2mm またはICB-97B 1.6mm)を推奨します。
2. 使用マイコンはArduino MicroのArduino純正品です。それ以外では正しく動作しないかピン配置が異なります。ご注意ください。
3. 10kΩの集合抵抗は8素子タイプで、コモンを1とした場合は2番目と3番目のピンをカットしてください。
4. 添付の[回路図通り](ME38_MC53_Pedal_SWBox_DG_VM_V4.1.0.7.pdf)に[製作](ME38_MC53_Pedal_SWBox_DG_VM_V4.1.0.7.png)します。
   [回路図](ME38_MC53_Pedal_SWBox_DG_VM_V4.1.0.7.pdf)と[実態配線図](ME38_MC53_Pedal_SWBox_DG_VM_V4.1.0.7.png)がありますので参考にしてください。
   コネクタ類も参考です。直接配線しても問題はありません。
5. AE-MCP4725の基板裏面はSDAとSCLのプルアップのみハンダでジャンパー接続します。A0はピンに配線しています。
6. Arduino IDEには、Adafruit MCP23017 Arduino Library と Adafruit_MCP4725 Arduino Library を導入してください。

> [!WARNING]
>- 配線が1本でも異なると所望の動作はしません。SPIとI2Cのバスライン、+5VとGNDのショートは特に注意してください。
>- 本開発品によるいかなる破損や不具合等が生じても当方はいかなる責任を負いかねますので、ご了承ください。
>- BU0836を使用した外部機器とは相性不具合を起こす場合があります。ご注意ください。

## BVEとの連動方法
> [こちら](https://github.com/GraphTechKEN/SerialOutputEx)を参照してください。

## 謝辞
本開発品はED67900-5様の[実物車両運転シミュレータ製作講座 | ツナギ図 laboratory](https://tsunagi-labo.booth.pm/items/3547706)を参考にしております。  
この場を借りて御礼申し上げます。
