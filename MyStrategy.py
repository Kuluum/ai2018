from model.action import Action
from model.game import Game
from model.robot import Robot
from model.rules import Rules

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

    def __truediv__(self, num: float):
        return Vector2D(self.x / num, self.z / num)

    def __str__(self):
        return '({0}, {1})'.format(self.x, self.z)

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

    def __str__(self):
        return '({0}, {1}, {2})'.format(self.x, self.z, self.y)

    def dot(self, other):
        return abs(self.x * other.x + self.z * other.z + self.y * other.y)

    def normalize(self):
        return Vector3D(self.x/self.len(), self.z/self.len(), self.y/self.len())

    def D2(self):
        return Vector2D(self.x, self.z)


class MyStrategy:

    game: Game
    rules: Rules

    attacker: Robot
    defender: Robot

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
        self.attacker = me
        gate_center = 0
        gate_z_target = rules.arena.depth / 2.0 + game.ball.radius * 2

        if game.ball.z - self.game.ball.radius * 3 > me.z:
            self.should_hit = True
        elif game.ball.z < me.z:
            self.should_hit = False

        if self.should_hit:

            jump_predict = self.predict_jump(me, tiks=15)  # if jump right now
            for i in range(0, 15):
                jp = jump_predict[i]
                bp = self.last_prediction[i]
                dist_to_ball = (jp - bp).len()
                jump = (dist_to_ball <= BALL_RADIUS + ROBOT_MAX_RADIUS / 2 and jp.z < game.ball.z - game.ball.radius - ROBOT_MAX_RADIUS / 2)
                if jump:
                    action.jump_speed = ROBOT_MAX_JUMP_SPEED
                    return

            '''
            if (Vector2D(game.ball.x, game.ball.z) - Vector2D(me.x, me.z)).len() < ROBOT_MAX_GROUND_SPEED * 0.75:
                tiks_pass = 0
                for point in self.last_prediction[:45]:
                    gate_center_direction = Vector2D(gate_center - point.x, gate_z_target - point.z).normalize()
                    ball_to_center_point = Vector3D(point.x - gate_center_direction.x * game.ball.radius,
                                                    point.z - gate_center_direction.z * game.ball.radius,
                                                    point.y)
                    delta_pos = ball_to_center_point - me
                    target_velocity = delta_pos.normalize() * ROBOT_MAX_GROUND_SPEED

                    poses = self.predict_move(me, target_velocity, tiks=tiks_pass + 1)
                    if (poses[tiks_pass].D2() - point.D2()).len() <= self.game.ball.radius + me.radius:

                        jump_predict = self.predict_jump(me, tiks=min(tiks_pass + 1, 10), target_vel=target_velocity)
                        jump = False
                        for i in range(0, min(tiks_pass + 1, 10)):
                            jp = jump_predict[i]
                            bp = self.last_prediction[i]
                            dist_to_ball = (jp-bp).len()
                            jump = (dist_to_ball <= BALL_RADIUS + ROBOT_MAX_RADIUS and jp.z < game.ball.z - game.ball.radius)
                            if jump:
                                break

                        action.target_velocity_x = target_velocity.x
                        action.target_velocity_y = 0.0
                        action.target_velocity_z = target_velocity.z
                        action.jump_speed = ROBOT_MAX_JUMP_SPEED if jump else 0.0
                        action.use_nitro = False
                        #print('predicted')
                        return
                    tiks_pass += 1
            '''
            point = self.last_prediction[0]
            gate_center_direction = Vector2D(gate_center - point.x, gate_z_target - point.z).normalize()
            ball_to_center_point = Vector3D(point.x - gate_center_direction.x * game.ball.radius,
                                            point.z - gate_center_direction.z * game.ball.radius,
                                            1)
            delta_pos = ball_to_center_point - me
            if delta_pos.len() <= ROBOT_MAX_RADIUS:
                delta_pos = point - me

            target_velocity = delta_pos.normalize() * ROBOT_MAX_GROUND_SPEED
            action.target_velocity_x = target_velocity.x
            action.target_velocity_z = target_velocity.z
            return

        else:  # Run to take place between our gate and ball
            tiks_pass = 0
            for point in self.last_prediction:
                tiks_pass += 1
                if point.y > self.game.ball.radius * 5:
                    continue

                target_x = point.x
                if target_x - me.x < 0:
                    target_x += game.ball.radius * 2
                else:
                    target_x -= game.ball.radius * 2

                jump_predict = self.predict_jump(me)
                jump = False
                for i in range(1, 15):
                    jp = jump_predict[i]
                    bp = self.last_prediction[i]
                    dist_to_ball = ((jp.x - bp.x) ** 2
                                    + (jp.y - bp.y) ** 2
                                    + (jp.z - bp.z) ** 2
                                    ) ** 0.5
                    jump = (dist_to_ball <= BALL_RADIUS + ROBOT_MAX_RADIUS and jp.z < game.ball.z - game.ball.radius)
                    if jump:
                        break

                target_z = point.z - game.ball.radius * 3

                target = Vector3D(target_x, target_z, 1)

                delta_pos = target-Vector3D(me.x, me.z, me.y)
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
        self.defender = me
        gate_center = 0
        gate_z_target = rules.arena.depth / 2.0 + game.ball.radius * 2
        gate_center_direction = Vector2D(gate_center - game.ball.x, gate_z_target - game.ball.z).normalize()


        jump_predict = self.predict_jump(me, tiks=15)  # if jump right now
        for i in range(0, 15):
            jp = jump_predict[i]
            bp = self.last_prediction[i]
            dist_to_ball = ((jp.x - bp.x) ** 2
                            + (jp.y - bp.y) ** 2
                            + (jp.z - bp.z) ** 2
                            ) ** 0.5
            jump = (dist_to_ball <= BALL_RADIUS + ROBOT_MAX_RADIUS/2 and jp.z < game.ball.z - game.ball.radius - ROBOT_MAX_RADIUS/2 and jp.y <= bp.y)
            if jump:
                action.jump_speed = ROBOT_MAX_JUMP_SPEED
                action.use_nitro = False
                return

        tiks_pass = 0
        for point in self.last_prediction:
            if point.z < -20 and me.z < point.z:
                ball_to_center_point = Vector3D(point.x - gate_center_direction.x * game.ball.radius,
                                                point.z - gate_center_direction.z * game.ball.radius,
                                                point.y - game.ball.radius)

                delta_pos = ball_to_center_point - me
                need_speed = ROBOT_MAX_GROUND_SPEED
                target_velocity = delta_pos.normalize() * need_speed

                poses = self.predict_move(me, target_velocity, tiks=tiks_pass+1)

                if (poses[tiks_pass] - point).len() <= self.game.ball.radius + me.radius and point.y < game.ball.radius + me.radius:
                    action.target_velocity_x = target_velocity.x
                    action.target_velocity_z = target_velocity.z
                    return
                tiks_pass += 1

        target_pos = Vector2D(
            0.0, -(rules.arena.depth / 2.0) - 2.5)
        target_velocity = Vector2D(
            target_pos.x - me.x, target_pos.z - me.z) * ROBOT_MAX_GROUND_SPEED
        action.target_velocity_x = target_velocity.x
        action.target_velocity_y = 0.0
        action.target_velocity_z = target_velocity.z

    def predict_ball(self, sec: int = 1, tik: int = 60, tiks: int = 0):
        ball_x = self.game.ball.x
        ball_z = self.game.ball.z
        ball_y = self.game.ball.y
        ball_vel_x = self.game.ball.velocity_x
        ball_vel_z = self.game.ball.velocity_z
        ball_vel_y = self.game.ball.velocity_y
        ball_rad = self.game.ball.radius

        distances = Helpers()
        ball_positions = [Vector3D(ball_x, ball_z, ball_y)]
        num = tiks+1 if tiks > 0 else sec*tik
        for i in range(1, num):

            if ball_y - ball_rad >= 0 and ball_y + ball_rad < self.rules.arena.height:
                ball_vel_y = ball_vel_y - GRAVITY / float(tik)
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

    def predict_jump(self, me: Robot, sec: int = 1, tik: int = 60, tiks: int = 0, target_vel:Vector3D = None):
        pos = Vector3D(me.x, me.z, me.y)
        vel = target_vel if target_vel else Vector3D(me.velocity_x, me.velocity_z, ROBOT_MAX_JUMP_SPEED)
        jump_pos = [pos]

        num = tiks+1 if tiks > 0 else sec*tik
        for i in range(1, num):
            pos += vel / float(tik)
            if pos.y > ROBOT_MAX_RADIUS:
                vel.y -= GRAVITY / float(tik)
            jump_pos.append(pos)
        return jump_pos

    def predict_move(self, me: Robot, target_vel: Vector3D, sec: int = 1, tik: int = 60, tiks: int = 0, show_vel: bool = False):
        pos = Vector3D(me.x, me.z, me.y)
        vel = Vector3D(me.velocity_x, me.velocity_z, me.velocity_y)
        poses = []
        vels = []
        num = tiks+1 if tiks > 0 else sec*tik
        for i in range(1, num):
            target_vel_change = target_vel - vel
            if target_vel_change.len() > 0:
                accel = 100
                vel += target_vel_change.normalize() * accel / float(tik)
            if vel.len() > ROBOT_MAX_GROUND_SPEED:
                vel = vel.normalize() * ROBOT_MAX_GROUND_SPEED
            pos += vel / float(tik)
            if show_vel:
                vels.append((pos, vel))
            poses.append(pos)
        if show_vel:
            return poses, vels
        else:
            return poses

    def custom_rendering(self):
        predictions = self.predict_ball(1, 60)
        ajumps = self.predict_jump(self.attacker)
        djumps = self.predict_jump(self.defender)

        spheres_str = []
        for point in predictions[:20]:
            gate_center = 0
            gate_z_target = self.rules.arena.depth / 2.0 + self.game.ball.radius * 2
            sphere = '{{"Sphere":{{"x": {0}, "y": {1}, "z": {2}, "radius": {3}, "r": 1.0, "g": 1.0, "b": 1.0, "a": 0.5}}}}'.format(point.x, point.y, point.z, self.game.ball.radius)
            l = '{{"Line":{{"x1": {0}, "y1": {1}, "z1": {2}, "x2": {3}, "y2": {4}, "z2": {5}, "width": 1.0, "r": 0.0, "g": 1.0, "b": 0.0, "a": 0.5}}}}'.format(point.x, point.y, point.z, gate_center, 2, gate_z_target)
            spheres_str.append(sphere)
            spheres_str.append(l)

        for j in ajumps[:15]:
            sphere = '{{"Sphere":{{"x": {0}, "y": {1}, "z": {2}, "radius": {3}, "r": 1.0, "g": 0.0, "b": 0.0, "a": 0.5}}}}'.format(j.x, j.y, j.z, ROBOT_MAX_RADIUS)
            spheres_str.append(sphere)

        for j in djumps[:15]:
            sphere = '{{"Sphere":{{"x": {0}, "y": {1}, "z": {2}, "radius": {3}, "r": 0.0, "g": 0.0, "b": 1.0, "a": 0.5}}}}'.format(j.x, j.y, j.z, ROBOT_MAX_RADIUS)
            spheres_str.append(sphere)

        result = "[" + ','.join(spheres_str) + "]"
        return result


class Helpers:

    def dan_to_plane(self, point: Vector3D, point_on_plane: Vector3D, plane_normal: Vector3D):
        return (point-point_on_plane).dot(plane_normal) / plane_normal.len()

