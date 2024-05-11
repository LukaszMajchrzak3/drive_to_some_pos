# drive_to_some_pos
## Opis paczki
Za pomocą tej paczki możliwe jest sterowanie platformą Husky (a200_0000) - użytkownik podaje zadaną lokalizację, do której następnie zmierza platforma. Node sterujący działaniem został stworzony w języku C++.
## Instalacja
1. Przejdź do swojego workspace'a, do którego zamierzasz przenieść paczkę, a następnie do folderu src.
2. Użyj polecenia "git clone https://github.com/LukaszMajchrzak3/drive_to_some_pos.git", wyjdź z folderu src i załaduj paczkę poleceniem "colcon build --packages-select drive_to_some_pos".
3. Wykonaj ruch przy użyciu polecenia "ros2 run drive_to_some_pos drive_to_pos_node --ros-args -p pos_x:=1.0 -p pos_y:=1.0". Zmienne "pos_x" oraz "pos_y" określają lokalizacje, do której udać ma się robot.
