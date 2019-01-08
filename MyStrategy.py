from model.action import Action
from model.game import Game
from model.robot import Robot
from model.rules import Rules
from model.arena import Arena

EPS = 1e-5

BALL_RADIUS = 2.0
ROBOT_MAX_RADIUS = 1.05
MAX_ENTITY_SPEED = 100.0
ROBOT_MAX_GROUND_SPEED = 30.0
ROBOT_MAX_JUMP_SPEED = 15.0
GRAVITY = 30
JUMP_TIME = 0.2
MAX_JUMP_HEIGHT = 3.0


class Vector2D:

    def __init__(self, x=0.0, z=0.0):
        self.x = x
        self.z = z

    def len(self):
        return ((self.x * self.x) + (self.z * self.z))**0.5

    def __sub__(self, other):
        return Vector2D(self.x - other.x, self.z - other.z)

    def __add__(self, other):
        return Vector2D(self.x + other.x, self.z + other.z)

    def __mul__(self, num: float):
        return Vector2D(self.x * num, self.z * num)

    def normalize(self):
        return Vector2D(self.x/self.len(), self.z/self.len())


class Vector3D:

    def __init__(self, x=0.0, z=0.0, y=0.0):
        self.x = x
        self.z = z
        self.y = y

    def len(self):
        return ((self.x * self.x) + (self.z * self.z) + (self.y * self.y))**0.5

    def __sub__(self, other):
        return Vector3D(self.x - other.x, self.z - other.z, self.y - other.y)

    def __add__(self, other):
        return Vector3D(self.x + other.x, self.z + other.z, self.y + other.y)

    def __mul__(self, num: float):
        return Vector3D(self.x * num, self.z * num, self.y * num)

    def __truediv__(self, num: float):
        return Vector3D(self.x / num, self.z / num, self.y / num)

    def dot(self, other):
        return abs(self.x * other.x + self.z * other.z + self.y * other.y)

    def normalize(self):
        return Vector3D(self.x/self.len(), self.z/self.len(), self.y/self.len())


class MyStrategy:

    game: Game
    rules: Rules

    attacker: Robot
    should_hit = False
    ataker_target = Vector3D(0, 0, 0)
    last_prediction = []
    sec = 1
    tik = 60

    def act(self, me: Robot, rules: Rules, game: Game, action: Action):
        self.game = game
        self.rules = rules
        is_attacker = len(game.robots) == 2

        for robot in game.robots:
            robot: Robot = robot
            if robot.is_teammate and robot.id != me.id:
                if robot.z < me.z:
                    is_attacker = True

        self.last_prediction = self.predict_ball(self.sec, self.tik)

        if is_attacker:
            self.act_attacker(me, rules, game, action)
        else:
            self.act_defender(me, rules, game, action)


    def act_attacker(self, me: Robot, rules: Rules, game: Game, action: Action):
        #print('x: {0} z: {1} vel_x:{2} vel_z: {3}'.format(me.x, me.z, me.velocity_x, me.velocity_z))
        self.attacker = me
        dist_to_ball = ((me.x - game.ball.x) ** 2
                        + (me.y - game.ball.y) ** 2
                        + (me.z - game.ball.z) ** 2
                        ) ** 0.5

        # If ball between robot and enemy's gates and robot hit the bal after jump then jump to hit the ball.
        jump = (dist_to_ball <= BALL_RADIUS +
                ROBOT_MAX_RADIUS * 4 and me.z < game.ball.z)

        gate_center = rules.arena.width / 2.0
        gate_z_target = rules.arena.depth + game.ball.radius * 2

        ball_curr_x = game.ball.x
        ball_curr_z = game.ball.z

        if ball_curr_z - self.game.ball.radius * 3 > me.z:
            self.should_hit = True
        elif ball_curr_z < me.z:
            self.should_hit = False


        if self.should_hit:
            gate_center_direction = Vector2D(gate_center - ball_curr_x, gate_z_target - ball_curr_z).normalize()
            tiks_pass = 0
            for point in self.last_prediction:
                tiks_pass += 1
                ball_to_center_point = Vector2D(point.x - gate_center_direction.x * (game.ball.radius + me.radius),
                                                point.z - gate_center_direction.z * (game.ball.radius + me.radius))
                self.ataker_target = Vector3D(ball_to_center_point.x, ball_to_center_point.z, 1)

                delta_pos = ball_to_center_point - me
                need_speed = delta_pos.len() / self.tik * tiks_pass

                if  need_speed < ROBOT_MAX_GROUND_SPEED and \
                    need_speed > 0.7 * ROBOT_MAX_GROUND_SPEED:
                    target_velocity = delta_pos.normalize() * need_speed
                    action.target_velocity_x = target_velocity.x
                    action.target_velocity_y = 0.0
                    action.target_velocity_z = target_velocity.z
                    action.jump_speed = ROBOT_MAX_JUMP_SPEED if jump else 0.0
                    action.use_nitro = False
                    return
        else:  # Run to take place between our gate and ball
            tiks_pass = 0
            for point in self.last_prediction:
                tiks_pass += 1
                if point.y > self.game.ball.radius * 4:
                    continue

                target_x = point.x
                #if target_x - me.x < 0:
                #    target_x += game.ball.radius * 2
                #else:
                #    target_x -= game.ball.radius * 2

                target_z = point.z - game.ball.radius * 3

                target = Vector3D(target_x, target_z, 1)


                delta_pos = target-Vector3D(me.x, me.z, me.y)
                delta_len = delta_pos.len()
                need_speed = ROBOT_MAX_GROUND_SPEED
                target_velocity = delta_pos.normalize() * need_speed

                action.target_velocity_x = target_velocity.x
                action.target_velocity_y = 0.0
                action.target_velocity_z = target_velocity.z
                action.jump_speed = ROBOT_MAX_JUMP_SPEED if jump else 0.0
                self.ataker_target = target
                return
        return

    def act_defender(self, me: Robot, rules: Rules, game: Game, action: Action):
        dist_to_ball = ((me.x - game.ball.x) ** 2
                        + (me.y - game.ball.y) ** 2
                        + (me.z - game.ball.z) ** 2
                        ) ** 0.5

        # If ball between robot and enemy's gates and robot hit the bal after jump then jump to hit the ball.
        jump = (dist_to_ball < BALL_RADIUS +
                ROBOT_MAX_RADIUS*4 and me.z < game.ball.z)

        target_pos = Vector2D(
            0.0, -(rules.arena.depth / 2.0) + rules.arena.bottom_radius)

        if game.ball.velocity_z < -EPS:

            t = (target_pos.z - game.ball.z) / game.ball.velocity_z
            x = game.ball.x + game.ball.velocity_x * t

            if abs(x) < rules.arena.goal_width / 2.0:
                target_pos.x = x

        target_velocity = Vector2D(
            target_pos.x - me.x, target_pos.z - me.z) * ROBOT_MAX_GROUND_SPEED

        action.target_velocity_x = target_velocity.x
        action.target_velocity_y = 0.0
        action.target_velocity_z = target_velocity.z
        action.jump_speed = ROBOT_MAX_JUMP_SPEED if jump else 0.0
        action.use_nitro = False

    def predict_ball(self, sec: int = 1, tik: int = 60):
        ball_x = self.game.ball.x
        ball_z = self.game.ball.z
        ball_y = self.game.ball.y
        ball_vel_x = self.game.ball.velocity_x
        ball_vel_z = self.game.ball.velocity_z
        ball_vel_y = self.game.ball.velocity_y
        ball_rad = self.game.ball.radius

        distances = Helpers()
        ball_positions = []
        for i in range(1, sec*tik):

            if ball_y - ball_rad >= 0 and ball_y + ball_rad < self.rules.arena.height:
                ball_vel_y = ball_vel_y - GRAVITY / 60.0
            elif abs(ball_vel_y) > EPS:
                ball_vel_y = -ball_vel_y * 0.7

            side_x_dist = distances.dan_to_plane(Vector3D(ball_x, ball_z, ball_y),
                                                 Vector3D(self.rules.arena.width / 2.0, 0, 0), Vector3D(-1, 0, 0))
            if side_x_dist - ball_rad <= 0 or side_x_dist + ball_rad >= self.rules.arena.width:
                ball_vel_x = -ball_vel_x * 0.7

            side_z_dist = distances.dan_to_plane(Vector3D(ball_x, ball_z, ball_y),
                                                 Vector3D(0, self.rules.arena.depth / 2.0, 0), Vector3D(0, -1, 0))
            if side_z_dist - ball_rad <= 0 or side_z_dist + ball_rad >= self.rules.arena.depth:
                ball_vel_z = -ball_vel_z * 0.7

            ball_x = ball_x + ball_vel_x / float(tik)
            ball_z = ball_z + ball_vel_z / float(tik)
            ball_y = ball_y + ball_vel_y / float(tik)
            ball_positions.append(Vector3D(ball_x, ball_z, ball_y))
        return ball_positions

    def custom_rendering(self):
        predictions = self.predict_ball(2, 60)

        spheres_str = []
        for point in predictions:
            sphere = '{{"Sphere":{{"x": {0}, "y": {1}, "z": {2}, "radius": {3}, "r": 1.0, "g": 1.0, "b": 1.0, "a": 0.5}}}}'.format(point.x, point.y, point.z, self.game.ball.radius)
            spheres_str.append(sphere)

            ataker_pos = Vector3D(self.attacker.x, self.attacker.z, self.attacker.y)
        result = "[" + ','.join(spheres_str) + \
                 ', {{ "Line": {{"x1": {0}, "z1": {1}, "y1": {2}, ' \
                 '"x2": {3}, "z2": {4}, "y2": {5},' \
                 '"width": 1.0, "r":1.0, "g":1.0, "b":1.0, "a":1.0 }} ' \
                 '}}'.format(ataker_pos.x, ataker_pos.z, ataker_pos.y,
                             self.ataker_target.x, self.ataker_target.z, self.ataker_target.y) + "]"
        return result


class Helpers:

    def dan_to_plane(self, point: Vector3D, point_on_plane: Vector3D, plane_normal: Vector3D):
        return (point-point_on_plane).dot(plane_normal) / plane_normal.len()

    def angle_vecters(self, a:Vector3D, b:Vector3D):
        import math
        if a.len() == 0 or b.len() == 0:
            return 0
        arccosInput = a.dot(b) / a.len() / b.len()
        arccosInput = 1.0 if arccosInput > 1.0 else arccosInput
        arccosInput = -1.0 if arccosInput < -1.0 else arccosInput
        return math.acos(arccosInput)


'''
    def attacker_position_rate(self):
        ball_gate_side = self.game.ball.z - self.attacker.z
        ball_gate_side_rate = 0 if ball_gate_side > 0 else ball_gate_side

        x_distance_rate = - abs(self.game.ball.x - self.attacker.x)
        z_distance_rate = - abs(ball_gate_side)

        gate_center = self.rules.arena.width / 2.0
        gate_z_target = self.rules.arena.depth + self.game.ball.radius * 2

        gate_center_direction = Vector3D(gate_center - self.game.ball.x, gate_z_target - self.game.ball.z, 0).normalize()
        attacker_velocity = Vector3D(self.attacker.velocity_x, self.attacker.velocity_z, 0).normalize()
        velocity_angle_rate = 360 - self.angle_vecters(gate_center_direction, attacker_velocity)
        attacker_velocity_len = attacker_velocity.len()

        return 10 * ball_gate_side_rate + x_distance_rate + z_distance_rate #+ velocity_angle_rate
'''