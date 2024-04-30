import cv2
import numpy as np
import mediapipe as mp
import pygame
import heapq

mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands

def calculate_3d_coordinates(results_hand, image_width, image_height):
    coordinates_3d = []
    for hand_landmarks in results_hand.multi_hand_landmarks:
        for landmark in hand_landmarks.landmark:
            x = int(landmark.x * image_width)
            y = int(landmark.y * image_height)
            coordinates_3d.append((x, y))
    return coordinates_3d

def euclidean_distance(p1, p2):
    return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2 + (p1[2] - p2[2]) ** 2) ** 0.5

def astar_algorithm(start, goal, obstacles):
    open_list = []
    closed_set = set()
    heapq.heappush(open_list, (0, start, []))  # (f_score, node, path)
    
    while open_list:
        f_score, current, path = heapq.heappop(open_list)
        if current == goal:
            return path + [current]
        
        if current in closed_set:
            continue
        
        closed_set.add(current)
        
        for neighbor in get_neighbors(current):
            if neighbor in closed_set or neighbor in obstacles:
                continue
            
            g_score = len(path) + 1  # Assume uniform cost
            h_score = euclidean_distance(neighbor, goal)
            f_score = g_score + h_score
            heapq.heappush(open_list, (f_score, neighbor, path + [current]))
    
    return None

def get_neighbors(point):
    x, y, z = point
    neighbors = [(x+1, y, z), (x-1, y, z), (x, y+1, z), (x, y-1, z), (x, y, z+1), (x, y, z-1)]  # 3D grid
    return [(nx, ny, nz) for nx, ny, nz in neighbors]

def main():
    hands = mp_hands.Hands(
        min_detection_confidence=0.7, min_tracking_confidence=0.7)
    cap = cv2.VideoCapture(0)
    
    pygame.init()
    screen = pygame.display.set_mode((640, 480))
    pygame.display.set_caption("3D Map with A* Pathfinding")
    clock = pygame.time.Clock()
    
    running = True
    while running:
        ret, image = cap.read()
        if not ret:
            continue
        image_width = image.shape[1]
        image_height = image.shape[0]
        image = cv2.flip(image, 1)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results_hand = hands.process(image)
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if results_hand.multi_hand_landmarks:
            coordinates_3d = calculate_3d_coordinates(results_hand, image_width, image_height)
            camera_position = (image_width // 2, image_height // 2, 0)  # Assuming camera at the center of the 3D space
            hand_position = coordinates_3d[8]  # Index finger position
            obstacles = []  # No obstacles for now
            path = astar_algorithm(camera_position, hand_position + (0,), obstacles)  # Convert hand position to 3D
            if path:
                screen.fill((255, 255, 255))
                for point in path:
                    pygame.draw.circle(screen, (255, 0, 0), (point[0], point[1]), 5)
                pygame.display.flip()
        
        cv2.imshow("Hand Tracking", image)
        if cv2.waitKey(1) & 0xFF == 27:
            break
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

    hands.close()
    cap.release()
    cv2.destroyAllWindows()
    pygame.quit()

if __name__ == '__main__':
    main()
