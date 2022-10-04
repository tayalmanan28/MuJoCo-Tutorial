# Tutorial de Mujoco 👓
Tutorial sobre cómo empezar con las simulaciones de MuJoCo.

Actualmente estamos participando en el Hacktober Fest 2022 🎃, por lo que si quieres contribuir a este repositorio, sigue las instrucciones dadas en la sección de [contribución](https://github.com/tayalmanan28/MuJoCo-Tutorial/blob/main/README.md#contributing) 😉
![image](https://user-images.githubusercontent.com/42448031/193699422-a75d4807-e7ab-456a-9f57-e82195647c3b.png)


## Instalación y puesta en marcha: 🚀

### Configuración del entorno Conda 🐍

Para instalar Anaconda siga las instrucciones de esta [página web](https://www.digitalocean.com/community/tutorials/how-to-install-the-anaconda-python-distribution-on-ubuntu-20-04-quickstart) (Ubuntu 20.04)

Creé un entorno conda para la configuración de MuJoCo: 
```
conda create --name mujoco-tut  
```
Cambie al entorno recién creado (notará el nombre del entorno en la línea de comandos en el extremo izquierdo):
```
conda activate mujoco-tut  
```

A continuación, clone el repositorio en su sistema:
```
git clone https://github.com/tayalmanan28/Mujoco-Tutorial.git
```
Instale los siguientes elementos necesarios:
```
pip install -r requirements.txt
```

### Instalación de MuJoCo 👨‍💻

Para instalar MuJoCo en su sistema, siga el siguiente [blog](https://tayalmanan28.github.io/my_blogs/mujoco/simulations/robotics/2022/01/21/MuJoCo.html)

### Ejemplo de funcionamiento 👉

``` python3 run.py ```

## Contenido

El propósito principal de este repositorio es proporcionar el código de inicio necesario para ejecutar una simulación MuJoCo con llamadas de teclado y ratón utilizando sus enlaces de Python. La clase base está en `mujoco_base.py`. 
Para crear tu propia simulación MuJoCo, puedes crear una nueva clase que herede de `mujoco_base.MuJoCoBase`. Un ejemplo de este uso se proporciona en `example_projectile.py`, la nueva clase debe implementar las siguientes funciones

```[Python]
- reset()       # Inicializa el entorno y las llamadas de control
- controller()  # Añade acciones de control
- simulate()    # Copiar la función simulate() de
                # mujoco_base.MuJoCoBase y añade tu propio toque
```

## Ejemplos de MuJoCo 👉


```[Markdown]
- Proyectil con arrastre
- Controlar un péndulo simple
- Controlar un péndulo doble
- Balanceo de la pierna
- Manipulator drawing
- Controlar un péndulo subactuado
- Columpio de gimnasta/suelta en una barra
- Tolva 2D
- Problema del valor inicial
- Cinemática inversa
- Bípedo 2D
```



## READMEs in different Languages

español: [Readme](https://github.com/tayalmanan28/MuJoCo-Tutorial/blob/main/README_sp.md)

## Contribución 🤝

Puedes contribuir a este repositorio de dos maneras:
1. Añadiendo nuevos ejemplos de entornos MuJoCo
2. Ayudando a resolver los problemas existentes

### Para contribuir con un nuevo ejemplo a este repo:

- Bifurque este repositorio. Puede bifurcar este repositorio haciendo clic en el botón de Fork en la esquina superior derecha. Una vez que se bifurca esto creará una copia del repositorio en su cuenta
- Siga los pasos anteriores para la instalación 
- Ve a la carpeta `examples` y revisa los diferentes ejemplos de entornos de mujoco.
- Cree un archivo xml válido. Las instrucciones para hacer un archivo XML se mencionan [aquí](https://mujoco.readthedocs.io/en/latest/overview.html?highlight=hello.xml#examples)
- Luego puedes usar uno de los entornos como base para crear un entorno mujoco para tu ejemplo y debatir si hay algún problema.
- Una vez completado, crea un pull request, lee sobre cómo enviar un pull request en el tutorial de DigitalOcean "
[Cómo crear un Pull Request en GitHub](https://www.digitalocean.com/community/tutorials/how-to-create-a-pull-request-on-github)".


### Para las contribuciones directamente de los Issues:

- Bifurque este repositorio. Puede bifurcar este repositorio haciendo clic en el botón de Fork en la esquina superior derecha. Una vez que se bifurca esto creará una copia del repositorio en su cuenta
- Siga los pasos anteriores para la instalación 
- Basándose en su experiencia, seleccione un problema del botón de problemas de arriba y pida que se le asigne el problema. Trabaje en el tema y coménte si tiene algún problema.
- Creé un pull request, lee sobre cómo enviar un pull request en el tutorial de DigitalOcean " [Cómo crear un pull request en GitHub](https://www.digitalocean.com/community/tutorials/how-to-create-a-pull-request-on-github)".

En breve, revisaremos su solicitud y la fusionaremos con la rama principal del proyecto si su solicitud es válida.  También recibirás una notificación cuando tu pull request se fusione con el código base existente. Después podrá ver sus detalles en la sección de colaboradores en la página de abajo.


## LICENCIA 📃

El código está bajo la licencia MIT y puede ser utilizado por cualquiera sin ninguna restricción. 👍
***

<p align='center'>Creado con ❤ por <a href="https://github.com/tayalmanan28">Manan Tayal</a> y traducido por <a href="[https://github.com/tayalmanan28](https://github.com/ArturoEmmanuelToledoAguado)">Arturo</a>

