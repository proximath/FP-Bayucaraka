# How To: Menjalankan Ros Nodes dan Integrasi Microros

## Step 0: Dependencies
Pastikan modules berikut terinstall
```
numpy
opencv-contrib-python
onnxruntime
ultralytics
```
Pastikan juga **ros2** (Humble) sudah terkonfigurasi dengan baik.

## Step 1: Kalibrasi Kamera
Ambil beberapa foto chessboard menggunakan kamera dan rename file dengan format `{index}.jpg` dengan index mulai dari 0.
Kemudian, jalankan script `programming/revisi/utils/get-mtx-and-dist.py`. Akan muncul matriks intrinsik `mtx` dan koefisien distorsi `dist`.

## Step 2: Ambil Dataset
Gunakan script `programming/revisi/utils/capture-dataset.py` untuk mengambil dataset. Script menghasilkan dua output, `{index}.mp4` dan `{index}_raw.mp4` pada directory `programming/dataset`. Selanjutnya, anotasikan dataset menggunakan platform seperti **Roboflow** dan export dengan format **YOLO11** dan image size `512x512`. Download zipnya dan extract pada directory `programming/revisi/annotated-dataset`.

## Step 3: Training
Pastikan annotated dataset sudah Gunakan script `programming/revisi/utils/train.py` untuk melatih model. Jika Anda ingin training lebih cepat, bisa mengganti argument `device` dari `"cpu"` menjadi `"gpu"`. Model

## Step 4: ROS Nodes
Jalankan node `camera` dan `crane` dengan script `programming/revisi/row_ws/run.sh`. Pastikan untuk memberikan izin eksekusi pada script dengan command `chmod +x run.sh`. Script bisa digunakan dengan format `./run.sh <node>`. Pastikan node sudah berjalan dengan perintah `ros2 node list`.

## Step 5: Jalankan Crane
Pastikan payload dan dropzone sudah berada di alas box. Kemudian jalankan crane dengan perintah `ros2 service call /start std_srvs/srv/Empty`. Robot akan memindai selama beberapa detik untuk mencari posisi payload dan dropzone. Kemudian, posisi keduanya akan dipublish ke topic `/move`. Anda bisa verifikasi data dengan perintah `ros2 topic echo /move`.

## Step 6: Micro ROS
**Pastikan micro_ros_agent sudah dibuild dan disource**
Upload code electrical pada MCU kemudian jalankan micro ros agent dengan perintah `ros2 run micro_ros_agent micro_ros_agent --dev /dev/ttyUSB0`. Pastikan sudah muncul tulisan **session connected**. Jika belum, bisa cabut dan pasang lagi kabel USB. Jika semua sudah benar, jalankan lagi crane dengan perintah `ros2 service call /start std_srvs/srv/Empty` dan robot seharusnya akan mulai berjalan.