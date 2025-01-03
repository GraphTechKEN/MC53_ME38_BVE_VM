# MC53-ME38_BVE_VM
- 本開発品は主に主に国鉄型SELD式車両向け等のマスコン、ブレーキ弁などの入力操作と速度計や電圧計のメーターなどの外部表示器をBVEと連動させるものです。

> [!TIP]
>- BVE5.8と6の両方に対応
>- Arduino Microを使用し、HIDキーボード機能を利用したキー入力と、シリアルポートを介した機器連動を同時に実現します。
>- BVEだけではなく、一部の鉄道シミュレーターなどの操作が可能(メーター類は不動)
>- 部品の配線やポテンショの追加改造のみで実装可能
>- 付属の[調整ソフト](https://github.com/GraphTechKEN/MC53_ME38_BVE_VM/releases/)で、ブレーキ弁角度や速度計補償などの細かい設定も可能
>- [こちら](https://github.com/GraphTechKEN/115-1000_Display)の115系1000番台向け表示灯拡張基板を製作し接続すると、メーターパネルも連動可能
>- ブレーキ弁へはポテンショ(可変抵抗)を工夫して取付けます。
>- 動作確認済みマスコン：MC37A、MC53、MC22、MC54、小田急1000形未更新
>- 動作確認済みブレーキ弁：ME38、ME40、ME48、ME50、ME62、ME72
>- 動作確認済みメーター：SR35、SR36 (Typ.10mA品)、小田急1000形未更新車は2kΩを直列に挟むこと

## 製作方法
~~実装済基板は[こちら](http://graphtechken.booth.pm/items/6223832)~~
1. 基板は秋月電子製片面紙エポキシ・ユニバーサル基板　Ａの小タイプ(AE-5)またはサンハヤト製(ICB-97 1.2mm またはICB-97B 1.6mm)を推奨します。
2. 使用マイコンはArduino MicroのArduino純正品です。それ以外では正しく動作しないかピン配置が異なります。ご注意ください。
3. 10kΩの集合抵抗は8素子タイプで、コモンを1とした場合は2番目と3番目のピンをカットしてください。
4. 添付の[回路図通り](ME38_MC53_Pedal_SWBox_DG_VM_V4.1.1.9.pdf)に[製作](ME38_MC53_Pedal_SWBox_DG_VM_V4.1.1.9.png)します。
   [回路図](ME38_MC53_Pedal_SWBox_DG_VM_V4.1.1.9.pdf)と[実態配線図](ME38_MC53_Pedal_SWBox_DG_VM_V4.1.1.9.png)がありますので参考にしてください。
   コネクタ類も参考です。直接配線しても問題はありません。
5. AE-MCP4725の基板裏面はSDAとSCLのプルアップのみハンダでジャンパー接続します。A0はピンに配線しています。(ジャンパーを忘れると正常動作しません)
6. Arduino IDEには、Adafruit MCP23017 Arduino Library と Adafruit_MCP4725 Arduino Library を導入してください。
   ![実態配線図](https://github.com/GraphTechKEN/MC53_ME38_BVE_VM/blob/main/ME38_MC53_Pedal_SWBox_DG_VM_V4.1.1.9.png)

> [!WARNING]
>- 配線が1本でも異なると所望の動作はしません。SPIとI2Cのバスライン、+5VとGNDのショートは特に注意してください。
>- 本開発品によるいかなる破損や不具合等が生じても当方はいかなる責任を負いかねますので、ご了承ください。
>- BU0836を使用した外部機器とは相性不具合を起こす場合があります。ご注意ください。

> [!TIP]
>- Serial1出力と5V出力は、表示灯類など拡張用のために用います。
>- Serial1出力(Tx)：USBからのSerial電文をそのままSerial1から他基板へ渡します。
>- Serial1入力(Rx):他基板(表示基板など)から本基板へ信号を渡します。
>- 331e線が実車ではGndですが、その配線ですと力行しない限りレバーサの前進(4線)と後進(5線)が判別できないため、331h線をGndとし、331e線を1段目に付け替えています。

## BVEとの連動方法
> [こちら](https://github.com/GraphTechKEN/SerialOutputEx)のSerialOutputExを参照してください。

## 謝辞
本開発品はツナギ図 laboratory(ED67900-5様代表ほか)の出版冊子[実物部品を使った運転シミュレータのつくりかた | ](https://booth.pm/ja/items/1756291)を参考にしております。  
この場を借りて御礼申し上げます。
本開発品は[まさき氏 X](https://x.com/ME48GEB1)により監修いただきました。この場を借りて御礼申し上げます。
本開発品は[万葉超音波温泉クハ115-1106](https://x.com/manyoonsen)の実車挙動を参考にさせていただきました。この場を借りて御礼申し上げます。

