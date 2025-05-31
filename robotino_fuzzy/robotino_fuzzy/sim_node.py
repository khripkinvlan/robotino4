#!/usr/bin/env python3

import pygame
import numpy as np
import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry


#import time

mm = 0.5 #Переменная для перевода из миллиметров в пиксели для вывода на экран
fps = 50
frame_time = 1/fps

def distance_between_points(point1_x, point1_y, point2_x, point2_y):
    return ((point1_x - point2_x) ** 2 + (point1_y - point2_y) ** 2)**0.5

pygame.init()

class SimParams:
    def __init__(self):
        
        self.display_size = (2000 * mm, 2000 * mm)

        #Настройка разрешения экрана
        self.screen = pygame.display.set_mode(self.display_size)

        #Название окна
        pygame.display.set_caption('Монстры со всех сторон')

        #victory_sound = pygame.mixer.Sound("sound\VictorySound.ogg")
        #defeat_sound = pygame.mixer.Sound("sound\DefeatSound.ogg")

        #channel1 = pygame.mixer.Channel(0)

        self.label = pygame.font.SysFont('Roboto', 50)
        self.lose_label = self.label.render('Томас уничтожен', False, 'Red')
        self.victory_label = self.label.render('Цель достигнута', False, 'Green')

        self.Thomas = Robot(0,0)
        self.missile_hitbox_diameter = int(70 * mm)
        self.target_diameter = int(120 * mm)
        self.Thomas.x = self.display_size[0]/2
        self.Thomas.y = (self.display_size[1]/mm - 50 - self.Thomas.diameter/(2*mm)) * mm

        self.running = True
        #robot_coordinates = starting_pos
        self.missiles = []
        self.missile_coordinates = (0,0)
        self.prev_spawn_time = 0

        #self.clock = pygame.time.Clock()

        self.target_coordinates = (0,0)
        self.attacking_missiles_amount = 0

        self.gameplay = True
        self.victory = False
        self.victory_flag = False
        self.defeat_flag = False
        self.start_flag = True

        self.walls = [
            Wall(0, 0, 2000 * mm, 0),  # Top
            Wall(0, 0, 0, 2000 * mm),  # Left
            Wall(2000 * mm, 0, 2000 * mm, 2000 * mm),  # Right
            Wall(0, 2000 * mm, 2000 * mm, 2000 * mm)   # Bottom
        ]
    def switch_reference_frame_to_relative (self, var, type = 0):
            if not type: # Если координаты
                relative_x = -1 * var[1] + (self.display_size[1]/mm - 50 - self.Thomas.diameter/(2*mm))
                relative_y = -1 * var[0] + 1000
            if type: # Если скорости или ускорения
                relative_x = (-1 * var[1])
                relative_y = (-1 * var[0])
            return(relative_x, relative_y)
    def switch_reference_frame_to_global (self, var, type = 0):
            if not type: # Если координаты
                relative_x = -1 * var[1] + 1000
                relative_y = -1 * var[0] + (self.display_size[1]/mm - 50 - self.Thomas.diameter/(2*mm))
            if type: # Если скорости или ускорения
                relative_x = -1 * var[1]
                relative_y = -1 * var[0]
            return(relative_x, relative_y)

        
    

class Robot:
    def __init__(self, x, y, diameter=450*mm, sensor_range=400*mm, sensor_count=9, max_velocity= 300, color = 'Red'):
            self.x = x
            self.y = y
            self.diameter = diameter
            self.radius = diameter / 2
            self.sensor_range = sensor_range
            self.sensor_count = sensor_count
            #self.sensors = np.linspace(90, 450, sensor_count, endpoint=False)  # Sensor angles
            self.sensors_readings = []
            for index in range(self.sensor_count):
                self.sensors_readings.append(sensor_range)
            self.sensors_rays = []
            self.x_velocity = 0
            self.y_velocity = 0
            self.max_velocity = max_velocity
            self.color = color

    def set_coordinate_x(self, x):
        self.x = x

    def set_coordinate_y(self, y):
        self.y = y

    def set_coordinates(self, x, y):
        self.x = x
        self.y = y

    def update_position(self):
        self.x = self.x + self.x_velocity * frame_time * mm
        self.y = self.y + self.y_velocity * frame_time * mm

    def draw(self,screen):
        pygame.draw.circle(screen, self.color, (self.x, self.y), self.diameter / 2)

    def wall_check(self):
        # ----------------Обработка столкновения со стеной---------------------
        if self.x < self.diameter / 2:
            self.x_velocity = 0
            self.x = self.diameter / 2

        elif self.x > 2000 * mm - self.diameter / 2:
            self.x_velocity = 0
            self.x = 2000 * mm - self.diameter / 2

        elif self.y < self.diameter / 2:
            self.y_velocity = 0
            self.y = self.diameter / 2

        elif self.y > 2000 * mm - self.diameter / 2:
            self.y_velocity = 0
            self.y = 2000 * mm - self.diameter / 2
        # ---------------------------------------------------------------------

    def velocity_limiter(self):
        if self.x_velocity**2 + self.y_velocity**2 > self.max_velocity**2:
            cor_vel = (self.x_velocity ** 2 + self.y_velocity ** 2)**0.5 - self.max_velocity
            cos_alpha = abs(self.x_velocity / (self.x_velocity ** 2 + self.y_velocity ** 2))
            sin_alpha = abs(self.y_velocity / (self.x_velocity ** 2 + self.y_velocity ** 2))
            self.x_velocity = np.sign(self.x_velocity)*(abs(self.x_velocity) - (cos_alpha * cor_vel))
            self.y_velocity = np.sign(self.y_velocity)*(abs(self.y_velocity) - (sin_alpha * cor_vel))

    def sensors_rays_init(self):
        angle_step = -(2 * np.pi/self.sensor_count)
        sensors_rays_start = []
        sensors_rays_end = []
        for sensor_index in range(self.sensor_count):
            sensors_rays_start.append((self.x + self.radius * np.cos(sensor_index*angle_step-np.pi/2), self.y + self.radius * np.sin(sensor_index*angle_step-np.pi/2)))
            end_x = sensors_rays_start[sensor_index][0] + self.sensors_readings[sensor_index] * np.cos(sensor_index*angle_step-np.pi/2)
            end_y = sensors_rays_start[sensor_index][1] + self.sensors_readings[sensor_index] * np.sin(sensor_index*angle_step-np.pi/2)
            sensors_rays_end.append((end_x,end_y))
            self.sensors_rays.append(((sensors_rays_start[sensor_index]),(sensors_rays_end[sensor_index])))


    def sensors_rays_calc(self):
        angle_step = -2 * np.pi/self.sensor_count
        sensors_rays_start = [0,0,0,0,0,0,0,0,0]
        sensors_rays_end = [0,0,0,0,0,0,0,0,0]
        for sensor_index in range(self.sensor_count):
            sensors_rays_start[sensor_index]=(self.x + self.radius * np.cos(sensor_index*angle_step-np.pi/2), self.y + self.radius * np.sin(sensor_index*angle_step-np.pi/2))
            end_x = sensors_rays_start[sensor_index][0] + self.sensors_readings[sensor_index] * np.cos(sensor_index*angle_step-np.pi/2)
            end_y = sensors_rays_start[sensor_index][1] + self.sensors_readings[sensor_index] * np.sin(sensor_index*angle_step-np.pi/2)
            sensors_rays_end[sensor_index]=(end_x,end_y)
            self.sensors_rays[sensor_index]=((sensors_rays_start[sensor_index]),(sensors_rays_end[sensor_index]))

    def sensors_rays_draw(self,screen):
    # Инициализация шрифта (если ещё не сделано)
        if not hasattr(self, 'font'):
            self.font = pygame.font.SysFont('Arial', 20)  # Шрифт Arial, размер 20
        
        for sensor_index in range(self.sensor_count):
            # Отрисовка луча (как раньше)
            pygame.draw.line(
                screen, 
                (255, 255, 0), 
                self.sensors_rays[sensor_index][0], 
                self.sensors_rays[sensor_index][1], 
                4
            )
            
            # Вычисление позиции для номера луча (на 25 пикселей раньше начала)
            start_point = np.array(self.sensors_rays[sensor_index][0])
            end_point = np.array(self.sensors_rays[sensor_index][1])
            direction = end_point - start_point
            direction_normalized = direction / np.linalg.norm(direction)  # Нормализуем вектор
            
            # Точка, смещённая на 25 пикселей от начала луча
            text_pos_index = start_point - 25 * direction_normalized
            text_pos_value = end_point - 25 * direction_normalized

            # Сдвигаем текст влево на 10 пикселей (корректировка по X)
            text_pos_index[0] -= 10  # Сдвиг номера луча
            
            # Рендерим текст с номером
            text_surface_index = self.font.render((str(sensor_index)+" ("+str(sensor_index+1)+")"), True, (0, 0, 0))  # Черный цвет
            screen.blit(text_surface_index, text_pos_index)  # Рисуем текст
            text_surface_value = self.font.render(str(round(self.sensors_readings[sensor_index]/mm,1)), True, (255, 255, 255))  # Белый цвет
            screen.blit(text_surface_value, text_pos_value)  # Рисуем текст


    def sensors_readings_calc(self, walls, missiles):
       # """Simulate ultrasonic sensor readings by checking walls and the moving obstacle."""
        search_radius = 1  # Smaller search radius for precision
        for sensor_index in range(self.sensor_count):
            start_x = self.sensors_rays[sensor_index][0][0]
            start_y = self.sensors_rays[sensor_index][0][1]

            angle_step = -(2 * np.pi / self.sensor_count)
            dx = np.cos(sensor_index*angle_step-np.pi/2)
            dy = np.sin(sensor_index*angle_step-np.pi/2)

            distance = self.sensor_range
            for step in np.linspace(0, int(self.sensor_range), num=100):  # Step along the ray
                test_x = start_x + step * dx
                test_y = start_y + step * dy

                # Check nearby points for any wall
                for wall in walls:
                    if wall.is_nearby(test_x, test_y, search_radius):
                        distance = min(distance, step)
                        #print(distance)
                        break

                # Check distance to the moving obstacle
                for missile in missiles:
                    if missile.is_nearby(test_x, test_y, search_radius):
                        distance = min(distance, step)
                        break
            self.sensors_readings[sensor_index] = distance

class Missile:
    def __init__(self, position, spawn_distance, velocity, robot, diameter=70*mm, color = 'Brown'):
        self.x = 0
        self.y = 0
        self.velocity = velocity
        self.diameter = diameter
        self.radius = diameter / 2
        self.x_velocity = 0
        self.y_velocity = 0
        self.position = position
        self.spawn_distance = spawn_distance
        self.color = color
        self.wall_collision = False
        self.is_attacking = True
        self.prev_distance = 0

        #-----------Расчет координат в декартовой, в зависимости от позиции и димтанции до робота_________
        angle = -self.position * 2*np.pi/9 + 3/2 * np.pi
        self.x = robot.x + ((self.diameter + robot.diameter)/2  + self.spawn_distance) * np.cos(angle)
        self.y = robot.y + ((self.diameter + robot.diameter) / 2 + self.spawn_distance) * np.sin(angle)

        self.x_velocity = velocity * -np.cos(angle)
        self.y_velocity = velocity * -np.sin(angle)

    def update_position(self):
        self.x = self.x + self.x_velocity * frame_time * 1000 * mm
        self.y = self.y + self.y_velocity * frame_time * 1000 * mm

    def draw(self,screen):
        pygame.draw.circle(screen, self.color, (self.x, self.y), self.diameter / 2)

    def interception_check(self, robot):
        if distance_between_points(robot.x, robot.y, self.x, self.y) < (self.diameter + robot.diameter) / 2:
            return True

    def is_nearby(self, px, py, radius):
        """Check if a point is within a certain radius of the obstacle."""
        return np.sqrt((px - self.x) ** 2 + (py - self.y) ** 2) <= self.radius + radius

    def wall_check(self):
        # ----------------Обработка столкновения со стеной---------------------
        if self.x < self.diameter / 2:
            self.x_velocity = 0                         #Левая стена
            self.x = self.diameter / 2
            self.wall_collision = True

        elif self.x > 2000 * mm - self.diameter / 2:
            self.x_velocity = 0                         #Правая стена
            self.x = 2000 * mm - self.diameter / 2
            self.wall_collision = True

        elif self.y < self.diameter / 2:
            self.y_velocity = 0                         #Верхняя стена
            self.y = self.diameter / 2
            self.wall_collision = True

        elif self.y > 2000 * mm - self.diameter / 2:
            self.y_velocity = 0                         #Нижняя стена
            self.y = 2000 * mm - self.diameter / 2
            self.wall_collision = True
        # ---------------------------------------------------------------------
    def attacking_status_check(self, robot):
        movement_vector = np.array([self.x_velocity, self.y_velocity])

        # Вектор от препятствия к роботу
        to_robot_vector = np.array([robot.x - self.x, robot.y - self.y])

        # Нормализуем векторы (получаем направление)
        if np.linalg.norm(movement_vector) == 0 or np.linalg.norm(to_robot_vector) == 0:
            return

        movement_direction = movement_vector / np.linalg.norm(movement_vector)
        to_robot_direction = to_robot_vector / np.linalg.norm(to_robot_vector)

        # Косинус угла между векторами (через скалярное произведение)
        dot_product = np.dot(movement_direction, to_robot_direction)

        # Угол в градусах
        angle = np.degrees(np.arccos(np.clip(dot_product, -1.0, 1.0)))

        # Если препятствие движется прямо на робота (маленький угол)
        if angle < 90:  # Порог атаки (можно настроить)
            self.is_attacking = True
        else:
            self.is_attacking = False

class Wall:
    def __init__(self, x1, y1, x2, y2):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2

    def is_nearby(self, px, py, radius):
        """Check if a point is within a certain radius of the wall."""
        if abs(self.x1 - self.x2) < 1e-6:  # Vertical wall
            # print(abs(px - self.x1))
            return abs(px - self.x1) <= radius # and min(self.y1, self.y2) <= py <= max(self.y1, self.y2)
        if abs(self.y1 - self.y2) < 1e-6:  # Horizontal wall
            # print(abs(py - self.y1))
            return abs(py - self.y1) <= radius # and min(self.x1, self.x2) <= px <= max(self.x1, self.x2)
        return False


class SimNode(Node):
    def __init__(self):
        super().__init__('sim_node')
        
        self.timer = self.create_timer(
            frame_time,
            self.timer_callback)
        self.Params = SimParams()
        self.get_logger().info("Node initialized with 120 Hz (120 second) timer")

        # Test publisher
        #self.test_pub = self.create_publisher(String, '/robotino4/test', 10)

        # Proximity sensors publisher
        self.sensors_pub = self.create_publisher(Float64MultiArray, '/robotino4/proximity_sensors', 10)

        # Odometry publisher
        self.odom_pub = self.create_publisher(Odometry, '/robotino4/odometry', 10)

        # Target coordinate publisher
        self.target_pos_pub = self.create_publisher(Point, '/robotino4/target_coordinate', 10)

        # Velocity command subscriber
        self.twist_sub = self.create_subscription(Twist, '/robotino4/cmd_vel', self.cmdvel_callback, 10)

        self.vx_cmd = 0.0
        self.vy_cmd = 0.0

        #self.prev_time = 0.0

    def timer_callback(self):
        try:
            #temp_time = time.time()
            #self.get_logger().info(str((temp_time-self.prev_time)*1000))
            #self.prev_time = temp_time
            #----------------------------------Subscriptions-----------------------------------------------------------------------------

            # ---------------------------------Simulator's code start--------------------------------------------------------------------------------------------------
            keys = pygame.key.get_pressed()
            if self.Params.gameplay:
                self.Params.screen.fill('black')
                if self.Params.start_flag:
                    # Загрузка музыки
                    # pygame.mixer.stop()
                    # pygame.mixer.music.load("sound\music\Vaginalnaya MaMBa - Street-cleaners theme.mp3")
                    # pygame.mixer.music.play(running)  # Воспроизведение музыки
                    self.Params.Thomas.x = self.Params.display_size[0]/2
                    self.Params.Thomas.y = (self.Params.display_size[1]/mm - 50 - self.Params.Thomas.diameter/(2*mm)) * mm
                    self.Params.Thomas.x_velocity = 0
                    self.Params.Thomas.y_velocity = 0
                    self.Params.Thomas.sensors_rays_init()
                    self.Params.target_coordinates = (self.Params.display_size[0]/2 + random.randint(int(-800*mm), int(800*mm)),self.Params.display_size[0]/2 + random.randint(int(-800*mm), int(800*mm)))
                    self.Params.prev_spawn_time = pygame.time.get_ticks()
                    self.Params.missiles.clear()

                    self.Params.start_flag = False

                if self.Params.Thomas.x_velocity or self.Params.Thomas.y_velocity:
                    self.Params.Thomas.velocity_limiter()
                self.Params.Thomas.update_position()
                self.Params.Thomas.draw(self.Params.screen)
                self.Params.attacking_missiles_amount = 0
                for missile in self.Params.missiles:
                    missile.attacking_status_check(self.Params.Thomas)
                    if missile.is_attacking:
                        self.Params.attacking_missiles_amount += 1

                if pygame.time.get_ticks() - self.Params.prev_spawn_time > 700 + random.randint(100, 400):
                    if self.Params.attacking_missiles_amount < 3:
                        pos = random.randint(0, 4)
                        if 2 < pos < 6:
                            pos += 4
                        self.Params.missiles.append(Missile(pos,400*mm, random.uniform(0.05,0.1),self.Params.Thomas))
                    prev_spawn_time = pygame.time.get_ticks()

                # print(attacking_missiles_amount)
                attacking_missiles_amount = 0
                self.Params.Thomas.sensors_readings_calc(self.Params.walls, self.Params.missiles)

                to_print = [0]*self.Params.Thomas.sensor_count
                for index in range(len(self.Params.Thomas.sensors_readings)):
                    value = self.Params.Thomas.sensors_readings[index]
                    to_print[index] = (value, ' ', index)
                #print(to_print)


                for i in self.Params.missiles:
                    i.update_position()
                    i.draw(self.Params.screen)

                self.Params.Thomas.sensors_rays_calc()
                self.Params.Thomas.sensors_rays_draw(self.Params.screen)
                # print(Thomas.x, '  ', Thomas.y)
                #missiles.clear()

                pygame.draw.circle(self.Params.screen, 'Green', self.Params.target_coordinates, self.Params.target_diameter / 2)
                pygame.display.update()  # Обновление экрана

                #Условие поражения
                for i in self.Params.missiles:
                    if i.interception_check(self.Params.Thomas):
                        self.Params.gameplay = False
                        self.Params.victory = False
                        self.Params.defeat_flag = True


                # Условие победы
                if (self.Params.Thomas.x-self.Params.target_coordinates[0])**2 + (self.Params.Thomas.y-self.Params.target_coordinates[1])**2 < (int(self.Params.target_diameter/2))**2:
                    self.Params.gameplay = False
                    self.Params.victory = True
                    self.Params.victory_flag = True

                # ----------------Обработка нажатя на клавиши---------------------
                self.Params.Thomas.x_velocity = self.vx_cmd
                self.Params.Thomas.y_velocity = self.vy_cmd
                if keys[pygame.K_UP]:
                    self.Params.Thomas.y_velocity += -0.1
                    #print(Thomas.x_velocity, Thomas.y_velocity)
                if keys[pygame.K_DOWN]:
                    self.Params.Thomas.y_velocity += 0.1
                    #print(Thomas.x_velocity, Thomas.y_velocity)
                if keys[pygame.K_LEFT]:
                    self.Params.Thomas.x_velocity += -0.1
                    #print(Thomas.x_velocity, Thomas.y_velocity)
                if keys[pygame.K_RIGHT]:
                    self.Params.Thomas.x_velocity += 0.1
                    #print(Thomas.x_velocity, Thomas.y_velocity)
                if keys[pygame.K_SPACE]:
                    if abs(self.Params.Thomas.x_velocity) < 0.15:
                        self.Params.Thomas.x_velocity = 0
                    else:
                        self.Params.Thomas.x_velocity += np.sign(self.Params.Thomas.x_velocity) * -0.15
                    if abs(self.Params.Thomas.y_velocity) < 0.015:
                        self.Params.Thomas.y_velocity = 0
                    else:
                        self.Params.Thomas.y_velocity += np.sign(self.Params.Thomas.y_velocity) * -0.015
                      
                if pygame.mouse.get_pressed()[0]:
                    self.Params.target_coordinates = pygame.mouse.get_pos()
                    

                self.Params.Thomas.wall_check()
                for (i, missile) in enumerate (self.Params.missiles):
                    # ----------------Обработка столкновения со стеной---------------------
                    if missile.x < missile.diameter / 2:
                        self.Params.missiles.pop(i)
                        del missile
                        # i.x_velocity = 0  # Левая стена
                        # i.x = i.diameter / 2

                    elif missile.x > 2000 * mm - missile.diameter / 2:
                        self.Params.missiles.pop(i)
                        del missile
                        # i.x_velocity = 0  # Правая стена
                        # i.x = 2000 * mm - i.diameter / 2

                    elif missile.y < missile.diameter / 2:
                        self.Params.missiles.pop(i)
                        del missile
                        # i.y_velocity = 0  # Верхняя стена
                        # i.y = i.diameter / 2

                    elif missile.y > 2000 * mm - missile.diameter / 2:
                        self.Params.missiles.pop(i)
                        del missile
                        # i.y_velocity = 0  # Нижняя стена
                        # i.y = 2000 * mm - i.diameter / 2


                # -----------------------------------------------------------------


            elif self.Params.victory:
                self.Params.screen.fill('white')
                self.Params.screen.blit(self.Params.victory_label,(self.Params.display_size[0]/2- 140,self.Params.display_size[1]/2 - 140))
                if self.Params.victory_flag:
                    self.Params.victory_flag = False
                    # pygame.mixer.music.load("sound\music\Victory.mp3")
                    # pygame.mixer.music.play()
                    # self.Params.channel1.play(self.Params.victory_sound)

                if keys[pygame.K_SPACE]:
                    self.Params.gameplay = True
                    self.Params.victory = False
                    self.Params.start_flag = True
            else:
                self.Params.screen.fill('darkgrey')
                self.Params.screen.blit(self.Params.lose_label,(self.Params.display_size[0]/2- 140,self.Params.display_size[1]/2 - 140))

                if self.Params.defeat_flag:
                    self.Params.defeat_flag = False
                    # pygame.mixer.music.load("sound\music\Defeat.mp3")
                    # pygame.mixer.music.play(start=1)
                    # self.Params.channel1.play(self.Params.defeat_sound)

                if keys[pygame.K_SPACE]:
                    self.Params.gameplay = True
                    self.Params.victory = False
                    self.Params.start_flag = True

            pygame.display.update()

            for event in pygame.event.get():
                if event.type == pygame.QUIT: #Обработка закрытия окна
                    self.Params.running = False
                    # pygame.mixer.music.stop()
                    pygame.quit()
            # ---------------------------------Simulator's code end--------------------------------------------------------------------------------------------------
            
            #----------------------------------Publications----------------------------------------------------

                # Prepare, publish and log sensors data
            sens_msg = Float64MultiArray()
            sens_msg.data = [0.0] * 9  # Initialize with zeros
            for i in range(9):
                sens_msg.data[i] = self.Params.Thomas.sensors_readings[i]/mm
            self.sensors_pub.publish(sens_msg)
            self.get_logger().info(
                f'Publishing sensor data: ' + ' '.join(f'{x:4.2f}' for x in sens_msg.data)
            )


                # Prepare, publish and log odometry data
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = 'odom'  # Parent frame
            odom_msg.child_frame_id = 'base_link'  # Child frame
            global_coordinates = (self.Params.Thomas.x/mm,self.Params.Thomas.y/mm)
            relative_coordinates = self.Params.switch_reference_frame_to_relative(global_coordinates)
            odom_msg.pose.pose.position.x = relative_coordinates[0]
            odom_msg.pose.pose.position.y = relative_coordinates[1]
            self.odom_pub.publish(odom_msg)
            self.get_logger().info(
                f'Publishing odometry data: x={odom_msg.pose.pose.position.x:5.4f}, y={odom_msg.pose.pose.position.y:5.4f}')
            
                    # Prepare, publish and log target position data
            target_point_msg = Point()
            target_point_global_coordinates = (self.Params.target_coordinates[0]/mm,self.Params.target_coordinates[1]/mm)
            target_point_relative_coordinates = self.Params.switch_reference_frame_to_relative(target_point_global_coordinates)
            target_point_msg.x = target_point_relative_coordinates[0]
            target_point_msg.y = target_point_relative_coordinates[1]
            target_point_msg.z = 0.0
            self.target_pos_pub.publish(target_point_msg)
            self.get_logger().info(
                f'Publishing target coordinates: x={target_point_msg.x:5.4f}, y={target_point_msg.y:5.4f}, z={target_point_msg.z:5.4f}')

        except Exception as e:
            self.get_logger().error(f"Error in timer callback: {str(e)}")

    def cmdvel_callback(self, msg):
        vx_relative = msg.linear.x
        vy_relative = msg.linear.y
        v_relative = (vx_relative,vy_relative)
        v_global = self.Params.switch_reference_frame_to_global(v_relative,'vel')
        self.vx_cmd = v_global[0]
        self.vy_cmd = v_global[1]
        return

def main(args=None):
    rclpy.init(args=args)
    
    node = SimNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()