# Chạy và chuẩn bị nộp bài

Gói nộp mới: `UEH_Nguyen-Hoang-Phuoc_submission.zip` (kèm `.sha256`). Báo cáo tiếng Anh 15 trang: `REPORT.pdf`; bản Word: `REPORT.docx`. Video mô phỏng: `SIMULATION_DEMO.mp4`, gần 5 phút, 30 fps. Lượt quay đạt 22,00 m odometry; chưa xác minh hoàn thành full map. Gói ZIP đã loại các tệp thử nghiệm và công cụ máy ảo.

Mã nguồn Windows: `D:\simulation`. Bản chạy Ubuntu: `/home/fish/crc_ws`.
Repository private: https://github.com/HFuoc/UEH_Nguyen-Hoang-Phuoc

## Xem mô phỏng

Mở VMware, vào Ubuntu và mở Terminal. Nếu đang chạy bộ đánh giá tự động, đợi hoàn tất trước khi đổi simulator.

```bash
cd ~/crc_ws
bash scripts/run_docker.sh down
bash scripts/run_docker.sh up
bash scripts/run_docker.sh sh
```

Trong shell container vừa mở, chạy:

```bash
ros2 launch crc_solution run.launch.py
```

Muốn xem camera, mở một Terminal Ubuntu khác:

```bash
cd ~/crc_ws
bash scripts/run_docker.sh sh
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

Không chạy đồng thời node `starter`, teleop và `crc_driver`. Nhấn Ctrl+C ở cửa sổ driver để dừng. Lệnh `down` xóa container; hãy xuất kết quả trước nếu cần giữ dữ liệu.

## Thay đổi tham số khi quay video

Trong một shell container khác:

```bash
ros2 param set /crc_driver max_speed 0.08
ros2 param get /crc_driver max_speed
```

Nói bằng lời của bạn: giảm tốc độ cực đại từ 0,18 xuống 0,08 m/s khiến robot đi chậm hơn. Thuật toán còn tự giảm tốc khi cua hoặc độ tin cậy thấp nên vận tốc thực tế có thể nhỏ hơn giá trị đặt.

Thay đổi bằng `ros2 param set` chỉ giữ trong lần chạy hiện tại. Nếu dừng driver rồi chạy lại để trình diễn, dùng `ros2 launch crc_solution run.launch.py max_speed:=0.08` để khởi động với tốc độ mới.

## Hiểu mã nguồn

- `perception.py`: tìm vạch qua nhiều hàng ảnh, ước lượng tâm làn và nhận dạng STOP/đèn; không biết tọa độ vật thể.
- `control.py`: quyết định dừng/chạy. STOP chỉ tính thời gian khi vận tốc đo gần bằng 0; khoảng chờ dùng thời gian mô phỏng.
- `node.py`: nhận cảm biến, chạy xử lý, phát `/cmd_vel`, ghi log và dừng khi mất dữ liệu. Đồng hồ kiểm tra lỗi dùng thời gian thực để vẫn dừng được khi mô phỏng bị pause.
- `analysis/summarize.py`: đọc CSV để vẽ kết quả. Quãng đường odometry không bằng tiến độ trên tuyến; sai lệch làn từ camera không phải điểm RMS của BTC.

## Hồ sơ còn cần bạn thực hiện

1. Đọc `REPORT.pdf`, kiểm tra kết quả và hoàn thiện khai báo phần bạn tự sửa/kiểm thử.
2. Quay video 6 phút liền mạch bằng tiếng Anh, có mặt trong góc, theo `VIDEO.md`; điền liên kết xem được vào file đó.
3. Xác nhận BTC còn nhận bài, cung cấp tài khoản giám khảo để thêm quyền truy cập repository private.
4. Nộp liên kết theo biểu mẫu của BTC. Chưa có thao tác tự gửi bài hoặc mời giám khảo trong phiên này.

Đây là bản cơ sở có kiểm thử, chưa hoàn thành cả đường và chưa có vượt xe. Xem `PROGRESS.md` để biết giới hạn và kết quả mới nhất.
