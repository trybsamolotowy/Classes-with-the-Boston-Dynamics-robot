import argparse
import os
import sys
import time
import bosdyn.client
import bosdyn.client.lease
import bosdyn.client.util
import bosdyn.geometry
from bosdyn.api import trajectory_pb2
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.client import math_helpers
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME,ODOM_FRAME_NAME, get_a_tform_b
from bosdyn.client.image import ImageClient
from bosdyn.client.robot_command import RobotCommandBuilder,RobotCommandClient, blocking_stand
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.util import seconds_to_duration
def hello_spot(config):

 # Skonfigurowanie informacji logowania
 bosdyn.client.util.setup_logging(config.verbose)
 # Inicjalizacja obiektu SDK stanowiącego połączenie z robotem.
 # Argument przekazywany do funkcji stanowi nazwę klienta.
 sdk = bosdyn.client.create_standard_sdk('HelloSpotClient')
 # Określenie adresu sieciowego robota
 # IP robota to 192.168.80.3
 robot = sdk.create_robot(config.hostname)
 # Uwierzytelnienie
 bosdyn.client.util.authenticate(robot)
 # Ustawienie synchronizacji czasu z robotem
 robot.time_sync.wait_for_sync()
 # Sprawdzenie czy robot nie jest w trybie E-Stop
 assert not robot.is_estopped(), 'Robot is estopped. Please use anexternal E-Stop client, such as the estop SDK example, to configure E-Stop.' # Uzyskanie informacji o stanie robota
robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
 # Przejęcie kontroli nad robotem
 # Tylko jeden klient może mieć kontrolę w danym momencie
lease_client = robot.ensure_client(bosdyn.client.lease.LeaseClient.default_service_name)
 # Utrzymanie kontroli nad robotem w przypadku rozłączenia
with bosdyn.client.lease.LeaseKeepAlive(lease_client,
must_acquire=True, return_at_exit=True):
 # Uruchomienie robota
 robot.logger.info('Powering on robot... This may take several seconds.')
 robot.power_on(timeout_sec=20)
 assert robot.is_powered_on(), 'Robot power on failed.'
 robot.logger.info('Robot powered on.')
 # Polecenie wstania
 robot.logger.info('Commanding robot to stand...')
 command_client = robot.ensure_client(RobotCommandClient.default_service_name)
 blocking_stand(command_client, timeout_sec=10)
 robot.logger.info('Robot standing.')
 time.sleep(3)
 # Uzyskanie informacji o stanie robota wraz z aktualną pozą
 robot_state = robot_state_client.get_robot_state()
 # Tryb stojący w zadanej pozycji.
 # W tym przykładzie RobotCommandBuilder generuje polecenie stój z rotacją inną niż domyślna względem "footprint frame".
 # układu nóg robota.
 footprint_R_body = bosdyn.geometry.EulerZXY(yaw=0.4, roll=0.0,
pitch=0.0)
cmd = RobotCommandBuilder.synchro_stand_command(footprint_R_body=footprint_R_body)
command_client.robot_command(cmd)
robot.logger.info('Robot standing twisted.')
time.sleep(3)
 # Obliczenie pożądanej pozycji i orientacji robota.
odom_T_flat_body = get_a_tform_b(robot_state.kinematic_state.transforms_snapshot,
ODOM_FRAME_NAME,
GRAV_ALIGNED_BODY_FRAME_NAME)
 # Wyznaczenie trajektorii do obrotu tułowia robota.

 # Określenie czasu dla każdej pozycji.
t1 = 2.5
t2 = 5.0
t3 = 7.5
# Wyznaczenie transformacji pożądanych pozycji.
flat_body_T_pose1 = math_helpers.SE3Pose(x=0.075, y=0, z=0,
rot=math_helpers.Quat())
flat_body_T_pose2 = math_helpers.SE3Pose(
x=0.0, y=0, z=0, rot=math_helpers.Quat(w=0.9848, x=0, y=0.1736,
z=0))
flat_body_T_pose3 = math_helpers.SE3Pose(x=0.0, y=0, z=0,
rot=math_helpers.Quat())
 # Wyznaczenie punktów trajektorii.
traj_point1 = trajectory_pb2.SE3TrajectoryPoint(
pose=(odom_T_flat_body * flat_body_T_pose1).to_proto(),
time_since_reference=seconds_to_duration(t1))
traj_point2 = trajectory_pb2.SE3TrajectoryPoint(
pose=(odom_T_flat_body * flat_body_T_pose2).to_proto(),
time_since_reference=seconds_to_duration(t2))
traj_point3 = trajectory_pb2.SE3TrajectoryPoint(
pose=(odom_T_flat_body * flat_body_T_pose3).to_proto(),
time_since_reference=seconds_to_duration(t3))
 # Zbudowanie trajektorii
traj = trajectory_pb2.SE3Trajectory(points=[traj_point1,
traj_point2, traj_point3])
 # Wyznaczenie parametrów ruchu.
body_control = spot_command_pb2.BodyControlParams(

body_pose=spot_command_pb2.BodyControlParams.BodyPose(root_frame_name=ODOM_
FRAME_NAME,

base_offset_rt_root=traj))
 # Przekazanie polecenia do robota.
robot.logger.info('Beginning absolute body control while standing.')
blocking_stand(command_client, timeout_sec=10,

params=spot_command_pb2.MobilityParams(body_control=body_control))
robot.logger.info('Finished absolute body control while standing.')
 # Przechwycenie obrazu.
image_client = robot.ensure_client(ImageClient.default_service_name)
 # Lista dostępnych źródeł wizyjnych.
sources = image_client.list_image_sources()
 # Wybranie lewej przedniej kamery RGB
image_response = image_client.get_image_from_sources(['frontleft_fisheye_image'])
 # Wyświetlenie obrazu
_maybe_display_image(image_response[0].shot.image)
if config.save or config.save_path is not None:
 # Zapis obrazu
 _maybe_save_image(image_response[0].shot.image,
config.save_path)
 # Dodanie komentarza w logach robota.
 log_comment = 'HelloSpot tutorial user comment.'
 robot.operator_comment(log_comment)
 robot.logger.info('Added comment "%s" to robot log.', log_comment)
 # Wyłączenie robota. Ustawienie parametru cut_immediately na False pozwala na wyłączenie robota w trybie bezpiecznym.
 robot.power_off(cut_immediately=False, timeout_sec=20)
 assert not robot.is_powered_on(), 'Robot power off failed.'
 robot.logger.info('Robot safely powered off.')
def _maybe_display_image(image, display_time=3.0):
 """Try to display image, if client has correct deps."""
 try:
 import io
 from PIL import Image
 except ImportError:
 logger = bosdyn.client.util.get_logger()
 logger.warning('Missing dependencies. Can\'t display image.')
 return
 try:
 image = Image.open(io.BytesIO(image.data))
 image.show()
 time.sleep(display_time)
 except Exception as exc:
 logger = bosdyn.client.util.get_logger()
 logger.warning('Exception thrown displaying image. %r', exc)
def _maybe_save_image(image, path):
 """Try to save image, if client has correct deps."""
 logger = bosdyn.client.util.get_logger()
 try:
 import io
 from PIL import Image
 except ImportError:
 logger.warning('Missing dependencies. Can\'t save image.')
 return
 name = 'hello-spot-img.jpg'
 if path is not None and os.path.exists(path):
 path = os.path.join(os.getcwd(), path)
 name = os.path.join(path, name)
 logger.info('Saving image to: %s', name)
 else:
 logger.info('Saving image to working directory as %s', name)
 try:
 image = Image.open(io.BytesIO(image.data))
 image.save(name)
 except Exception as exc:
 logger = bosdyn.client.util.get_logger()
 logger.warning('Exception thrown saving image. %r', exc)
def main(argv):
 """Command line interface."""
 parser = argparse.ArgumentParser()
 bosdyn.client.util.add_base_arguments(parser)
 parser.add_argument(
 '-s', '--save', action='store_true', help=
 'Save the image captured by Spot to the working directory. To chose
the save location, use --save_path instead.'
 )
 parser.add_argument(
 '--save-path', default=None, nargs='?', help=
 'Save the image captured by Spot to the provided directory. Invalid path saves to working directory.'
 )
 options = parser.parse_args(argv)
 try:
 hello_spot(options)
 return True
 except Exception as exc: # pylint: disable=broad-except
 logger = bosdyn.client.util.get_logger()
 logger.error('Hello, Spot! threw an exception: %r', exc)
 return False
if __name__ == '__main__':
 # Robota należy uruchomić z terminala w argumentach podając adres IP
robota (192.168.80.3).
 if not main(sys.argv[1:]):
 sys.exit(1)