# Tugas Magang: Mengintegrasikan OpenCV untuk Mendeteksi Bentuk dan Warna ke ROS Noetic
## Profil Penulis

Nama: Zico Diego Rio Ramadhonny

NRP : 5054251023

## Deskripsi
Pada tugas ini, saya mencoba mengimplementasikan salah satu library Python untuk pekerjaan computer vision, yaitu OpenCV, ke dalam ROS Noetic dan menyimulasikan secara sederhana bagaimana robot melihat lingkungan sekitar dengan bantuan OpenCV.

## Cara Pasang
Pertama, pastikan sudah memasang ROS Noetic.

Kedua, clone repositori ini.
```bash
git clone https://github.com/ZicoDiegoRR/tugas-magang-6-wayan.git
```

Ketiga, ubah working directory ke repositori.
```bash
cd <folder/ke/repositori>
```

Keempat, compile repositorinya menggunakan `catkin_make`.
```bash
catkin_make
```

Kelima, buat terminal baru dan jalankan `roscore`.
```bash
roscore
```

Keenam, kembali ke terminal sebelumnya dan source repositorinya.
```bash
source devel/setup.sh
```

Ketujuh, buat terminal baru lagi dan ulangi langkah keenam di dalam terminal baru.

Kedelapan, pilih salah satu terminal selain terminal untuk menjalankan `roscore` dan jalankan node ROS.
```bash
# NOTE: satu terminala hanya boleh menjalankan salah satu dari command di bawah ini!

rosrun opencv_ros client.py # Untuk klien
rosrun opencv_ros server # Untuk service server
```
