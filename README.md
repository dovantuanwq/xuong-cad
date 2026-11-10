# Xưởng CAD Karofi

Xem, vẽ, sửa và đo bản vẽ DWG/DXF; xem, đo, cắt mô hình 3D (STEP, STL…) ngay trong trình duyệt.
File bạn mở chỉ được xử lý trên máy bạn, không tải lên đâu cả.

Mở trang: `https://<tên-tài-khoản>.github.io/<tên-repo>/`

| File | Vai trò |
|---|---|
| `index.html` | Trang chính (các tab Bản vẽ 2D, Mô hình 3D, Dựng 3D với Claude, Hướng dẫn) |
| `2d.html`, `cad-worker.js`, `libredwg-web.wasm` | Bản vẽ 2D và bộ đọc DWG/DXF |
| `3d.html`, `step-worker.js` | Mô hình 3D và bộ đọc STEP |
| `huong-dan/` | Ảnh và PDF hướng dẫn |
| `mo-hinh/` | Mô hình mẫu |
| `cad2d3d.py` | Thư viện Python dựng 3D từ bản vẽ |
