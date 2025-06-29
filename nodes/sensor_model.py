from scipy import spatial
import numpy as np

class SensorModel:

    def __init__(self):
        self.ready = False
    
    def iniciar(self, occupancy_grid_msg):
        """
        occupancy_grid_msg : nav_msgs.msg.OccupancyGrid
        """
        self.map_data = occupancy_grid_msg
        self.width = occupancy_grid_msg.info.width
        self.height = occupancy_grid_msg.info.height
        self.resolution = occupancy_grid_msg.info.resolution

        self.mapa = self._process_map(occupancy_grid_msg)
        self.max_prob = self.gaussian_func(0)
        self.ready = True

    def _process_map(self, data):
        """
        Convierte el OccupancyGrid a una matriz 2D, mapea:
        Libre = 0, Ocupado = 1, Desconocido = -1
        """
        mapa = []
        ancho = data.info.width
        alto = data.info.height
        ocupied_threshold = 20
        free_threshold = 15

        n = 0
        fila = -1
        for elem in data.data:
            fila_actual = n // ancho
            if fila_actual != fila:
                fila = fila_actual
                mapa.append([])

            mapa[fila].append(elem)
            n += 1

        for i in range(alto):
            for j in range(ancho):
                if mapa[i][j] > ocupied_threshold:
                    mapa[i][j] = 1
                elif mapa[i][j] < free_threshold:
                    mapa[i][j] = 0
                else:
                    mapa[i][j] = -1
    
        self.kdtree = self._build_wall_kdtree(mapa)
        self.gaussian_func = self._gaussian(mean=0, std=0.02)

        probs = []
        for i in range(alto):
            probs.append([])
            for j in range(ancho): 
                dist, _ = self.kdtree.query([i, j])
                dist_metros = dist * self.resolution
                prob = self.gaussian_func(dist_metros)
                probs[i].append(prob)
        return probs

    def _build_wall_kdtree(self, mapa):
        """
        Crea un KDTree con los bordes de los muros para consultas rápidas de distancia
        """
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        intresting_walls = []
        for i in range(1, self.height - 1):
            for j in range(1, self.width - 1):
                if mapa[i][j] == 1:
                    if not all(mapa[i + di][j + dj] == 1 for di, dj in directions):
                        intresting_walls.append([i, j])

        return spatial.KDTree(intresting_walls)

    def _gaussian(self, mean, std):
        """
        Devuelve una función gaussiana
        """
        return lambda x: np.exp(-(x - mean) ** 2 / (2 * std ** 2)) / (np.sqrt(2 * np.pi) * std)

    def get_prob_at(self, world_x, world_y):
        """
        Devuelve la probabilidad de que una posición (x,y) en coordenadas del mundo sea coherente con el mapa.
        """
        map_x = int(world_x / self.resolution)
        map_y = int(world_y / self.resolution)

        if map_x < 0:
            map_x = 0 
        if map_y < 0:
            map_y = 0 
        if map_x >= self.width:
            map_x = self.width - 1
        if map_y >= self.height:
            map_y = self.height - 1

        return self.mapa[map_y][map_x]
