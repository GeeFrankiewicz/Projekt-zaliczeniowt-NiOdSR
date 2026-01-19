# UR5 ArUco Control Simulation

Projekt realizuje symulację robota Universal Robots UR5 w środowisku ROS 2 Humble. Sterowanie robotem odbywa się w czasie rzeczywistym za pomocą detekcji znaczników ArUco z kamery internetowej.

Całość środowiska jest zamknięta w kontenerze Docker, co gwarantuje działanie na każdym systemie Linux bez konieczności instalowania ROS-a na komputerze hosta.

## Funkcjonalności

* **Symulacja UR5:** Wykorzystanie sterownika `ur_robot_driver` w trybie `fake_hardware`.
* **Wizualizacja:** Podgląd robota w Rviz2 oraz dedykowany panel diagnostyczny z podglądem wizji.
* **Sterowanie wizyjne** odbywa się za sprawą wykrywania znacznika ArUco (Słownik 4x4, ID: 3) Umieszczenie znacznika powyżej środka obrazu daje obrót w prawo, a poniżej daje obrót w lewo.

## Wymagania Sprzętowe i Systemowe

1.  **System:** Linux (zalecane Ubuntu 20.04 lub nowsze).
2.  **Oprogramowanie:** Zainstalowany Docker
3.  **Kamera:** Kamera USB podłączona do komputera (domyślnie `/dev/video2`).
4.  **Znacznik:** Wydrukowany lub wyświetlony na telefonie znacznik **ArUco 4x4, ID 3**.

## Instrukcja Uruchomienia

### 1. Pobranie projektu
Sklonuj repozytorium i wejdź do katalogu głównego i wykonaj poniższe polecenia:
```bash
git clone <https://github.com/GeeFrankiewicz/Projekt-zaliczeniowt-NiOdSR>

cd ur5_aruco_project

docker build -t ur5_project_img .

xhost +local:docker

docker run -it --rm \
    --name ur5_aruco_container \
    --network host \
    --privileged \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $(pwd)/src:/ros2_ws/src \
    --device /dev/video2:/dev/video2 \
    ur5_project_img
```
