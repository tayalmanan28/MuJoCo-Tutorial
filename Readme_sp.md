# Mujoco Tutorial
Tutorial sobre cómo comenzar con las simulaciones de MuJoCo.

Actualmente también estamos participando en Hacktober Fest 2022, por lo que si desea contribuir a este repositorio, siga las instrucciones de contribución que se 
proporcionan en la Sección de contribución.
[below](https://github.com/tayalmanan28/MuJoCo-Tutorial/blob/main/README.md#contributing)
![image](https://user-images.githubusercontent.com/42448031/193699422-a75d4807-e7ab-456a-9f57-e82195647c3b.png)

## Instalación y primeros pasos:

### Configuración del entorno de Conda

Para instalar Anaconda siga las instrucciones en este [webpage](https://www.digitalocean.com/community/tutorials/how-to-install-the-anaconda-python-distribution-on
-ubuntu-20-04-quickstart) (Ubuntu 20.04)

Cree un entorno conda para la configuración de MuJoCo:
```
conda create --name mujoco-tut  
```
Cambie al entorno recién creado (observará el nombre del entorno en la línea de comando en el extremo izquierdo):
```
conda activate mujoco-tut  
```
Luego, clona el repositorio en tu sistema:
```
git clone https://github.com/tayalmanan28/Mujoco-Tutorial.git
```
Instale los siguientes paquetes necesarios:
```
pip install -r requirements.txt
```

### Instalación MuJoCo

Para instalar MuJoCo en su sistema, siga las [blog](https://tayalmanan28.github.io/my_blogs/mujoco/simulations/robotics/2022/01/21/MuJoCo.html)

### Ejemplo de ejecución

``` python3 run.py ```

## Contenido
El objetivo principal de este repositorio es proporcionar el código de inicio necesario para ejecutar una simulación de MuJoCo con devoluciones de llamada de
teclado y mouse utilizando sus enlaces de Python. La clase base está en `mujoco_base.py`. Para crear su propia simulación de MuJoCo, puede crear una nueva 
clase que herede `mujoco_base.MuJoCoBase`. Se proporciona un ejemplo de este uso en `example_projectile.py`, la nueva clase debería implementar las funciones


```[Python]
- reset()              # Inicializa el entorno y controla la devolución de llamada
- controller()        # Agrega acciones de control
- simulate()            # Copiar la función simular() de
                       #mujoco_base.MuJoCoBase y añade tu propio toque
```
## MuJoCo Examples


```[Markdown]
- Projectile with drag
- Control a simple pendulum
- Control a double pendulum
- Leg swing
- Manipulator drawing
- Control an underactuated pendulum
- Gymnast swing/release on a bar
- 2D Hopper
- Initial Value Problem
- Inverse Kinematics
- 2D Biped
```

## Contribuyendo

Así que puedes contribuir a este repositorio de 2 maneras:
1. Agregando nuevos ejemplos de entornos MuJoCo
2. Por ayuda en la resolución de los problemas existentes


### Por contribuir con un nuevo ejemplo a este repositorio:

- Bifurcar este repositorio. Puede bifurcar este repositorio haciendo clic en el botón bifurcar en la esquina superior derecha. Una vez que bifurques esto,
se creará una copia del repositorio en tu cuenta.
- Siga los pasos anteriores para la instalación
- Ve a la carpeta `examples` y revisa diferentes ejemplos del entorno mujoco.
- Cree un archivo xml válido. Las instrucciones para crear un archivo XML se mencionan [here](https://mujoco.readthedocs.io/en/latest/overview.html?
highlight=hello.xml#examples)
- Luego, puede usar uno de los entornos como base para crear un entorno mujoco para su ejemplo y discutir si hay algún problema.
- Una vez completado, cree una solicitud de extracción, lea sobre cómo enviar una solicitud de extracción en el tutorial de DigitalOcean "[Cómo crear una solicitud
de extracción en GitHub](https://www.digitalocean.com/community/tutorials/how-to-create-a-pull-request-on-github)".

### Para contribuciones directamente desde ediciones:

- Bifurcar este repositorio. Puede bifurcar este repositorio haciendo clic en el botón bifurcar en la esquina superior derecha. Una vez que bifurques esto, 
se creará una copia del repositorio en tu cuenta.
- Siga los pasos anteriores para la instalación
- En función de su experiencia, seleccione un problema del botón de problemas de arriba y solicite que se le asigne el problema. Trabaje en el tema y discútalo
si tiene algún problema.
- Cree una solicitud de extracción, lea sobre cómo enviar una solicitud de extracción en el tutorial de DigitalOcean "[Cómo crear una solicitud de extracción
en GitHub](https://www.digitalocean.com/community/tutorials/how-to-create-a-pull-request-on-github)".

Próximamente, revisaremos su solicitud y fusionaremos sus solicitudes de extracción con la rama principal del proyecto si su solicitud de extracción es válida.
También recibirá una notificación una vez que su solicitud de extracción se fusione con la base de código existente. Después de eso, podrá ver sus detalles en 
la sección de contribuyentes en la página a continuación.

## LICENSE

The code is licenced under the MIT license and free to use by anyone without any restrictions.
***

<p align='center'>Created with :heart: by <a href="https://github.com/tayalmanan28">Manan Tayal</a> and contributed by: Vivek Maurya<br>
<a href="https://github.com/bchainbuidler">Blockchain Buidler</a>,
<a href="https://github.com/Guillermo">Guiller</a>

Read this file in English : [Readme_sp](https://github.com/tayalmanan28/MuJoCo-Tutorial/blob/main/README.md)

