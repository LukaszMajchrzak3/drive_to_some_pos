# drive_to_some_pos
## Opis paczki
Za pomocą tej paczki możliwe jest sterowanie platformą Husky (a200_0000) - użytkownik podaje zadaną lokalizację, do której następnie zmierza platforma.
## Instalacja
1. W swoim workspace'ie utwórz nowe repozytorium np. używając polecenie: ros2 pkg create --build-type ament_cmake --license Apache-2.0 drive_to_some_pos (najlepiej jakby nazwa paczki była taka sama jak w tej instrukcji).
2. Pobierz pliki z repozytorium.
3. Spróbuj zbudować paczkę w terminalu. Jeśli paczka nie chce się zbudować, powtórz wszystkie czynności jeszcze raz.
4. Uruchom instrukcję korzystając z polecenie: ros2 run drive_to_some_pos drive_to_pos_node --ros-args -p pos_x:=1.0 -p pos_y:=1.0 . Argumenty pos_x oraz pos_y określają odpowiednio współrzędną x oraz współrzędną y, które mają zostać osiągnięte przez platformę.
