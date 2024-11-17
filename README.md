# Aruco searching project

## Cel projektu
Celem projektu jest zaprojektowanie algorytmu poruszającego robotem Turtlebot3 z zamontowaną kamerą RGB tak, aby znalazł on i określił lokalizację wszystkich pięciu markerów Aruco rozmieszczonych losowo w środowisku.

## Działanie
Algorytm porusza robotem Turtlebot3 korzystając z metody *random walk*. Robot wykonuje ruch do przodu tak długo, aż nie napotka przed sobą przeszkody (wykrywa to na podstawie skanu laserowego). Po wykryciu przeszkody następuje obrót w randomowym kierunku i kontynuacja jazdy prosto. Algorytm wykonywany jest tak długo aż nie upłynie czas zadany podczas wywoływania akcji.

W trakcie ruchu program analizuje obraz z kamery i wyszukuje na nim markery Aruco. Jeżeli marker zostanie wykryty, następuje obliczenie jego pozycji (na podstawie znanego rozmiaru markera oraz macierzy kamery) a następnie obliczenie jego pozycji względem układu map na podstawie przekształcienia `\tf` publikowanego przez węzeł Nav2.

Po zakończeniu wykonywania akcji wyświetlane są odczytane pozycje markerów.

### Subskrybowane tematy
Do działania, serwer akcji wykorzystuje następujące tamaty:

 - `/scan`: Skanowanie przestrzeni za pomocą czujnika LIDAR w celu unikania przeszkód.
 - `/camera/image_raw`: Pobieranie obrazów z kamery do wykrywania markerów.
 - `/camera/camera_info`: Pobieranie informacji o kalibracji kamery (macierz kamery i współczynniki dystorsji).
 - `/tf`: Uzyskiwanie transformacji pomiędzy układami współrzędnych.

### Publikowane tematy
Serwer akcji publikuje wiadomości w następujących tematach:

 - `/cmd_vel`: Publikowanie wiadomości odpowiadających za ruch robota.

### Relacje między tematami
![Przepływ informacji w programie](./img/topic%20relations.png)


## Program
Program zaimplementowany jest jako serwer akcji w ROS2. 

### Plik .action
Plik definiujący akcję wygląda następująco:

```
float32 timeout
---
bool succeeded
float32[] locations_x
float32[] locations_y
---
float32[] partial_locations_x
float32[] partial_locations_y
```

Celem akcji, podawanym podczas jej wywoływania jest czas `timeout`, określający jak długo ma działać algorytm przeszukujący środowisko.

Wynikiem akcji jest informacja `succeeded` informująca o tym, czy udało się znaleźć wszystkie markery oraz macierze `locations_x locations_y` zawierające koordynaty znalezionych markerów.

Jako feedback zwracane są macierze `partial_locations_x partial_locations_y` zawierające aktualnie odczytane koordynaty markerów.

### Główny skrypt serwera akcji
Struktura głównego skryptu jest następująca:

 - `SearchArucoActionServer`

Główna klasa odpowiedzialna za uruchomienie serwera akcji i realizację jego funkcji.

 - Inicjalizacja `(__init__)`

    - Tworzenie serwera akcji.
    - Subskrypcje do tematów `/scan`, `/camera/image_raw`, `/camera/camera_info`.
    - Inicjalizacja publikatora prędkości na `/cmd_vel`.
    - Ustawienie obiektu `tf_buffer` do pracy z transformacjami.

 - `execute_callback(self, goal_handle)`

    - Logika działania robota: poruszanie się i zbieranie danych o markerach.
    - Publikowanie informacji zwrotnej (feedback) do klienta akcji.
    - Zwracanie końcowego wyniku (Result) po zakończeniu działania.

 - `image_callback(self, msg)`

     - Przetwarzanie obrazu z kamery.
     - Detekcja markerów ArUco za pomocą OpenCV.
     - Przekształcenie współrzędnych markerów z układu kamery na układ map.

 - `laser_callback(self, msg)`

    - Odczyt danych z LIDAR-a w celu unikania przeszkód.

 - `camera_info_callback(self, msg)`

     - Odczyt macierzy kamery i współczynników dystorsji.

 - `get_transform(self, target_frame, source_frame)`    

     - Pobranie transformacji między układami odniesienia z tematu `/tf`.

 - `quaternion_to_rotation_matrix(self, quaternion)`

     - Konwersja kwaternionu na macierz obrotu.

 - Funkcja `main()`

     - Inicjalizacja i uruchomienie serwera akcji w wątku wielowątkowym.

## Potencjalne problemy
Największym problemem przedstawionego rozwiązania jest mało optymalny algorytm przeszukiwania środowiska. Bazuje on na losowych ruchach robota, przez co możliwe jest że robot będzie wielokrotnie odwiedzać znane już lokacje a odwiedzenie innych miejsc może zająć bardzo dużo czasu.

Drugim problemem jest niedokładność pozycjonowania markerów wynikacjąca zarówno z niedokładnej ich lokalizacji na obrazie z kamery jak i z niedokładnej lokalizacji samego robota.

## Potencjalne modyfikacje
Rozwiązaniem pierwszego problemu byłoby wykorzystanie innego algorytmu przeszukiwania środowiska. Można wykorzystać przykładowo paczkę `Omar-Salem/auto_mapper` lub `DaniGarciaLopez/ros2_explorer`.

Drugi problem można by było rozwiązać korzystając z rachunków prawdopodobieństwa w celu estymacji położenia markerów na podstawie aktualnego odczytu i wcześniejszych pomiarów (aktualnie zwracany jest tylko ostatni pomiar pozycji).

## Przykładowy wynik
Podczas testów algorytmu uzyskano następujące wyniki:

| Prawdziwe lokalizacje markerów | Lokalizacje markerów określone przez algorytm |
|:------------------------------:|:---------------------------------------------:|
| ``` x: 16.30 y: 1.71 ```       | ``` x: 17.35 y: 1.41 ```                      |
| ``` x: 3.36 y: 3.81 ```        | ``` x: 7.13 y: 1.03 ```                       |
| ``` x: 8.14 y: 3.25 ```        | ``` x: 8.07 y: 3.04 ```                       |
| ``` x: 16.80 y: -1.70 ```      | ``` x: 16.67 y: -2.49 ```                     |
| ``` x: 3.62 y: -3.38 ```       | ``` x: 4.16 y: 2.58 ```                       |

Jak można zauważyć w powyższej tabeli, lokalizacje ustalone przez algorytm są zbliżone do rzeczywistych, aczkolwiek w niektórych przypadkach występują duże różnice wynikające z opisanych wcześniej problemów.

## Uruchomienie:

W celu uruchomienia projektu należy wywołać następujące komendy (każda w osobnym terminalu):

```bash
# Uruchomienie symulacji
ros2 launch arm05_sim world.launch.py
```

```bash
# Uruchomienie serwera akcji
ros2 run search_aruco_action search_aruco_action_server.py
```

```bash
# Wysłanie celu do serwera akcji
ros2 action send_goal --feedback /SearchAruco search_aruco_action/action/SearchArUco "{timeout: 100.0}"
```

## Zależności
Projekt wymaga zainstalowania następujących pakietów:

 - `rclpy`
 - `cv2`
 - `numpy`
 - `tf2_ros`
 - `cv_bridge`