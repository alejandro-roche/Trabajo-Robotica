# Proyecto de Curso - Fundamentos de Robótica 🤖

## Descripción 📜
Este proyecto se centra en el análisis y control de un brazo robótico de 3 grados de libertad (GDL). El trabajo cubre los aspectos fundamentales de la **cinemática** y la **dinámica** de robots, incluyendo el modelado de movimientos, la implementación de controladores y la simulación de resultados. El objetivo es demostrar cómo los conceptos teóricos se aplican a un sistema robótico real.

El robot utilizado en este proyecto tiene inicialmente 6 grados de libertad, pero se reduce a un modelo más simple de 3 grados para facilitar el análisis y las simulaciones.

## Contenido 📚

### 1. Introducción 📝
Este proyecto tiene como objetivo aplicar y validar los conocimientos adquiridos sobre análisis cinemático y dinámico en robótica. Se modela un robot de 6 grados de libertad, que posteriormente se reduce a 3 grados para simplificar la resolución de problemas y las simulaciones. Se implementan controladores cinemáticos y dinámicos para analizar la respuesta del sistema y su precisión.

### 2. Análisis Cinemático 🔄
En esta sección se analiza la relación entre las posiciones de las articulaciones del robot y su extremo. El análisis incluye:
- **Cinemática Directa**: Determina la posición y orientación del extremo del robot utilizando el algoritmo Denavit-Hartenberg.
- **Cinemática Inversa**: Calcula las configuraciones articulares necesarias para alcanzar una posición y orientación específicas.
- **Jacobianos**: Estudia la relación entre las velocidades articulares y la velocidad del extremo del robot, ayudando a identificar puntos singulares.

### 3. Análisis Dinámico ⚙️
En esta sección se analiza la relación entre las fuerzas y torques aplicados al robot y su movimiento:
- **Dinámica Inversa**: Utiliza la formulación de Newton-Euler para calcular las fuerzas y torques necesarios para generar los movimientos deseados.
- **Simulador de Dinámica**: Verifica el modelo dinámico utilizando simulaciones para comparar los resultados con un modelo de referencia.

### 4. Control Cinemático 🎮
Se diseñan controladores que permiten al robot seguir una trayectoria específica en el espacio cartesiano. Este control se basa en la geometría del movimiento, sin tener en cuenta las fuerzas físicas.

### 5. Control Dinámico 🔧
A diferencia del control cinemático, el control dinámico toma en cuenta la dinámica del robot (fuerzas y torques) para garantizar que el robot siga la trayectoria deseada, considerando las fuerzas físicas reales.

### 6. Controladores Propuestos 🛠️
Se implementan y analizan varios controladores, cada uno con características específicas:
- **Controlador PD descentralizado**: Controla cada articulación de manera independiente.
- **Controlador PD con compensación de gravedad**: Añade compensación de gravedad para reducir el esfuerzo necesario para mover el robot.
- **Controlador PID descentralizado**: Añade un control proporcional, integral y derivativo para mejorar el rendimiento.
- **Controlador de Par Calculado**: Compensa toda la dinámica del robot, desacoplando las articulaciones y utilizando un control PD para cada una.

### 7. Simulaciones y Resultados 📊
Cada tipo de controlador se simula en MATLAB/Simulink. Los resultados incluyen gráficos de las trayectorias seguidas por el robot y la evolución de las coordenadas articulares.

### 8. Verificación ✅
Se verifica que los resultados obtenidos en las simulaciones sean correctos, utilizando un conjunto de funciones de validación. Se comparan los resultados con el modelo esperado para asegurarse de que el sistema funciona correctamente.

## Requisitos 🖥️
- **MATLAB** con **Simulink**.
- Archivos de funciones y modelos de simulación disponibles en este repositorio.

