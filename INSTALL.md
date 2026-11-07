# Réinstaller le robot

Procédure pour repartir d'une carte SD neuve. État de référence : Raspberry Pi 3B+,
Raspberry Pi OS 64 bits (Debian 13 trixie), utilisateur `robot`, code dans `~/Projects`.

## 1. Système

Avec Raspberry Pi Imager : utilisateur `robot`, SSH activé, Wi-Fi configuré.
L'adresse du robot est `192.168.1.3`.

Dans `/boot/firmware/config.txt`, ajouter ou vérifier :

```
dtparam=i2c_arm=on,i2c_arm_baudrate=40000   # PiStorms : I2C lent, obligatoire
dtparam=spi=on
dtoverlay=rpi-display,speed=32000000,rotate=90,dc_pin=18,reset_pin=22   # écran /dev/fb1
```

L'utilisateur `robot` doit être dans les groupes `dialout` (LiDAR), `i2c`, `spi`,
`gpio`, `video` (caméra, écran) et `input` (écran tactile) :

```bash
sudo usermod -aG dialout,i2c,spi,gpio,video,input robot
```

## 2. Code et environnement Python

```bash
cd ~/Projects
git clone -b package-imports https://github.com/Noc1a/Pistorms.git PiStorms
# copier aussi depuis le Mac : PersonFollower/, tests/, unitTests/, requirements.txt

# --system-site-packages : smbus et picamera2 viennent des paquets du système
python3 -m venv --system-site-packages ~/Projects/env
source ~/Projects/env/bin/activate
pip install -e "PiStorms[robot]" --config-settings editable_mode=compat
pip install -r requirements.txt     # ahrs, rplidar-roboticia, matplotlib
```

## 3. Services système

```bash
sudo ~/Projects/PiStorms/system/install.sh
```

Le script installe tout le dossier `system/` (à relancer après toute modification) :

| Fichier | Rôle |
|---|---|
| `pistorms-poweroff` → `/usr/lib/systemd/system-shutdown/` | À la fin d'un `poweroff`, coupe l'alimentation de la carte PiStorms (pas au reboot). |
| `psm_batt_monitor.service` | Surveille la batterie : LEDs bleues clignotantes sous 6,4 V, extinction sous 6,0 V. |
| `99-rplidar.rules` | Crée `/dev/rplidar` et démarre `lidar-pause` dès que le LiDAR est branché. |
| `lidar-pause.service` | Garde le moteur du LiDAR arrêté tant qu'aucun programme ne l'utilise. |

## 4. Vérifier

```bash
systemctl status psm_batt_monitor lidar-pause   # les deux "active (running)"
ls -l /dev/rplidar                              # -> ttyUSB0
python3 -m unittest discover unitTests          # tous OK
python3 tests/testScreen.py                     # écran et tactile
```

## Pourquoi `lidar-pause` existe

Le moteur du RPLidar A1 tourne quand la ligne DTR de son adaptateur USB (CP2102)
est inactive, et le pilote Linux la désactive à chaque fermeture du port : sans
programme, le moteur tourne. Le service garde le port ouvert avec DTR actif ;
les programmes LiDAR ouvrent le port en même temps et pilotent le moteur
eux-mêmes (`start_motor()` / `stop_motor()`).

Après un programme planté sans `stop_motor()`, le moteur continue de tourner :
`sudo systemctl restart lidar-pause` l'arrête. Si le plantage a eu lieu en plein
scan, le programme suivant peut échouer une fois avec `Descriptor length
mismatch` : il suffit de le relancer. Journal : `journalctl -u lidar-pause`.
