# 🤖 Sistema de Juego de Posturas para Niños con TDAH (ROS 2 + Visión por Computadora)

Este proyecto implementa un sistema interactivo en ROS 2 que utiliza un robot humanoide y una cámara para guiar a niños en un juego de memoria con posturas. Está orientado a apoyar terapéuticamente a niños con TDAH, mediante imitación de movimientos, motivación verbal y detección de emociones.

---

## 🚀 Funcionalidades principales

- Juego de memoria por secuencias con posturas corporales.
- Detección de posturas mediante cámara y MediaPipe.
- Feedback auditivo con sistema de Text-to-Speech (TTS).
- Registro emocional con inteligencia artificial.
- Guardado de estadísticas por niño para análisis posterior.

---

## 📦 Requisitos del sistema

Antes de instalar, asegúrate de tener:

### 🔧 ROS 2 (probado en Humble)

[Guía oficial de instalación](https://docs.ros.org/en/humble/Installation.html)

### 🖥️ Dependencias del sistema:

```bash
sudo apt update
sudo apt install \
  ros-humble-usb-cam \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-launch \
  python3-colcon-common-extensions
