# Máy may lập trình tự làm

Dự án học lập trình nhúng: tự lắp bo điều khiển cho máy may 1 kim, dùng encoder đọc góc kim, điều khiển bàn kẹp X–Y bằng motor bước và nâng chân vịt bằng solenoid.

## Có gì trong repo

| File | Nội dung |
|---|---|
| [`may-may-buoc-1/may-may-buoc-1.ino`](may-may-buoc-1/may-may-buoc-1.ino) | Code Arduino bước 1: đồng bộ bàn X–Y với góc kim, file mẫu may hình vuông 20 mm, solenoid "đá rồi giữ" |
| [`mo-hinh-may-may-3d.html`](mo-hinh-may-may-3d.html) | Mô hình 3D tương tác: kéo kim tới góc bất kỳ để xem lệnh firmware và biến nào đang chạy |
| [`linh-kien-may-may.html`](linh-kien-may-may.html) | Danh sách linh kiện cần mua, link shop Việt Nam, Taobao, 1688 |

## Xem mô hình 3D

Tải file `mo-hinh-may-may-3d.html` về rồi mở bằng Chrome hoặc Edge. Máy cần có mạng để tải thư viện 3D (three.js).

## Phần cứng

- Arduino Uno R3 + CNC Shield V3 + 2 driver A4988
- 2 motor bước NEMA 17, nguồn tổ ong 24V 5A, mạch hạ áp LM2596 xuống 12V
- Encoder LPD3806-600BM gắn 1:1 vào trục vô lăng
- Cảm biến quang chữ U FC-03 làm mốc kim
- Mạch MOSFET cách ly + solenoid 12V + diode 1N4007

## Đấu dây trên CNC Shield

| Linh kiện | Chân |
|---|---|
| Encoder kênh A / B | X− (D9) / Y− (D10) |
| Cảm biến chữ U | Abort (A0) |
| MOSFET solenoid | Z− (D11, PWM) |
| Motor X / Y | Cụm X / cụm Y |

**An toàn:** không nối dây nào vào hộp điều khiển motor servo của máy may. Bên trong có điện 310V.
