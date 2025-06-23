# lab3_ws
Repo para le lab 3 de rob movil

**Aclararacion, no se esta subiendo al repo log, install y build, porque siempre genera error con los path, por lo que solo se envia la informacion de los paquetes, nodos, launch, etc. Por esa misma razon es importante hacer install/setup.bash y colcon build siempre**

- 1. Clonar el repo: Desde la terminal de ubuntu, asegurarse de estar en el archivo raiz o en el directorio donde quieras clonar el WORKSPACE y ejecutar el siguiente comando:
```bash
git clone https://github.com/Mikson16/lab3_ws.git
```
- 2. Build de todas las dependencias necesarias:
```bash
cd lab3_ws
source install/setup.bash #Siempre hacer esto antes de compilar sinog en la lista no saldra el pkg
colcon build
```
- 3. Revisar la existencia de paquetes:
```bash
ros2 pkg list | grep lab3
```
