# SmartHouseX

> ภาคเรียนที่ 1 ปีการศึกษา 2567

เป็นส่วนหนึ่งของโครงงาน วิชา 01236256 Microcontroller and Embedded Systems

## รายชื่อสมาชิก

- 66010375 ธันยมัย จิตต์ณรงค์
- 66011314 กฤษณ์ เกษมเทวินทร์

## บทคัดย่อ TH

โครงงาน `SmartHouseX` บ้านอัจฉริยะที่คุมได้ทุกมิติ ถูกพัฒนาขึ้นเพื่อแก้ไขปัญหาคุณภาพอากาศที่ส่งผลกระทบต่อสุขภาพ เช่น ฝุ่น PM 2.5 รวมถึงปัญหาการใช้พลังงานอย่างสิ้นเปลืองและการดูแลสภาพแวดล้อมในบ้านให้อยู่ในสภาพที่เหมาะสม โดยใช้เทคโนโลยี `Internet of Things (IoT)` เพื่อสร้างระบบที่สามารถตรวจวัดและควบคุมอุปกรณ์ต่าง ๆ ภายในบ้านได้อย่างอัตโนมัติ โดยมี `Raspberry Pi`, `STM32NUCLEO-G071RB`, และ `ESP32` เป็นส่วนประกอบหลักในการทำงานร่วมกัน ซึ่ง `Raspberry Pi` ทำหน้าที่เป็นเซิร์ฟเวอร์หลักในการควบคุมอุปกรณ์ต่าง ๆ ภายในบ้านผ่านระบบ `Home Assistant` โดยสามารถแสดงสถานะของอุปกรณ์และข้อมูลจากเซ็นเซอร์ต่าง ๆ โดยผู้ใช้งานสามารถควบคุมระบบผ่านหน้าจอสัมผัสหรือแอปพลิเคชันมือถือ และเข้าถึงข้อมูลจากระยะไกลผ่าน `Cloudflare Tunnel` เพื่อให้ผู้ใช้งานสามารถตรวจสอบและควบคุมสภาพแวดล้อมในบ้านได้อย่างสะดวก ส่วน `STM32` ทำหน้าที่อ่านข้อมูลจากเซ็นเซอร์ `DHT11`, `PMS7003` และ `LDR` ข้อมูลที่ได้จะถูกส่งไปยัง `Raspberry Pi` เพื่อแสดงผลและควบคุมอุปกรณ์ไฟฟ้า นอกจากนี้ `ESP32` ใช้สำหรับการควบคุมระยะไกล ซึ่งผลลัพธ์จากการพัฒนาระบบแสดงให้เห็นว่า `SmartHouseX` ช่วยให้ผู้ใช้งานสามารถควบคุมอุปกรณ์ภายในบ้านได้อย่างสะดวก ลดการใช้พลังงาน และสามารถตรวจสอบคุณภาพอากาศและสภาพแวดล้อมภายในบ้านได้อย่างมีประสิทธิภาพ ซึ่งโครงงานนี้มีศักยภาพในการพัฒนาต่อยอดให้เป็นโซลูชันบ้านอัจฉริยะที่เหมาะสมกับการใช้ชีวิตในยุคดิจิทัลที่เน้นความสะดวกสบายและการดูแลสุขภาพ

> คำสำคัญ: `บ้านอัจฉริยะ`, `Internet of Things (IoT)`, `การควบคุมอัตโนมัติ`, `Raspberry Pi`, `STM32`, `ESP32`, `คุณภาพอากาศ`, `ประหยัดพลังงาน`, `Home Assistant`

For more information, see the documentation [here](/SmartHouseX.pdf).
