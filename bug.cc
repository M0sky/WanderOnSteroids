#include "stage.hh"
using namespace Stg;

static const double velocidad = 0.4;
static const double v_evasion = 0.05;
static const double g_evasion = 0.5;
static const double minfrontdistance = 0.5;
static const double tolerancia = 0.1;
static const bool debug = true;

typedef struct {
  ModelPosition *pos;
  ModelRanger *laser;
  Pose goal;
  bool goal_reached;
} robot_t;

int LaserUpdate(Model *mod, robot_t *robot);

extern "C" int Init(Model *mod, CtrlArgs *) {
  robot_t *robot = new robot_t();

  robot->pos = dynamic_cast<ModelPosition *>(mod);
  if (!robot->pos) {
    PRINT_ERR("No position model given in Bug controller.");
    exit(1);
  }
  robot->pos->Subscribe();
  robot->goal_reached = false;
  robot->goal = Pose(2.0, 2.0, 0.0, 0.0);
  
  World *world = mod->GetWorld();
  Model *test_model = new Model(world, nullptr, "test_model");
  Pose test_pose(robot->goal.x, robot->goal.y, 1.0, 0.0);
  test_model->SetPose(test_pose);
  test_model->SetColor(Color(0.0, 1.0, 0.0));
  world->AddModel(test_model);

  // Buscar sensor de rango
  ModelRanger *laser = NULL;
  for (int i = 0; i < 16; i++) {
    char name[32];
    snprintf(name, 32, "ranger:%d", i);
    laser = dynamic_cast<ModelRanger *>(robot->pos->GetChild(name));
    if (laser && laser->GetSensors()[0].sample_count > 8) {
      break;
    }
  }

  if (!laser) {
    PRINT_ERR("No se encontró un láser adecuado.");
    exit(2);
  }

  robot->laser = laser;
  robot->laser->AddCallback(Model::CB_UPDATE, model_callback_t(LaserUpdate), robot);
  robot->laser->Subscribe();

  return 0;
}

bool atGoal(robot_t *robot) {
  Pose current = robot->pos->GetPose();
  double dx = robot->goal.x - current.x;
  double dy = robot->goal.y - current.y;
  return (sqrt(dx * dx + dy * dy) < tolerancia);
}

int LaserUpdate(Model *, robot_t *robot) {

  if (robot->goal_reached)	
    return 0;

  // Check meta
  if (atGoal(robot)) {
    if (debug)
      printf("Objetivo alcanzado!\n");
    robot->goal_reached = true;
    robot->pos->SetXSpeed(0.0);
    robot->pos->SetTurnSpeed(0.0);
    robot->pos->Unsubscribe();
    
    // Detengo la simulación
    World *world = robot->pos->GetWorld();
    world->Stop();
    
    return 0;
  }

  // Datos del láser LIDAR (lecturas en metros)
  const std::vector<meters_t> &scan = robot->laser->GetSensors()[0].ranges;
  uint32_t num_muestras = scan.size();
  if (num_muestras < 1)
    return 0;

  bool obstaculo = false;
  double minleft = 1e6;
  double minright = 1e6;

  for (uint32_t i = 0; i < num_muestras; i++) {
    if (scan[i] < minfrontdistance) {
      obstaculo = true;
    }
    if (i > num_muestras / 2)
      minleft = std::min(minleft, scan[i]);
    else
      minright = std::min(minright, scan[i]);
  }

  if (obstaculo) {
    Pose current = robot->pos->GetPose();
    if (debug)
      printf("Posición actual: (%.2f, %.2f, ángulo: %.2f) | Obstáculo detectado!!!\n",current.x, current.y, current.a);
      
    robot->pos->SetXSpeed(v_evasion);

    if (minleft < minright) {
      robot->pos->SetTurnSpeed(-g_evasion);
    } else {
      robot->pos->SetTurnSpeed(+g_evasion);
    }
  } else {
    Pose current = robot->pos->GetPose();
    if (debug)
      printf("Posición actual: (%.2f, %.2f, ángulo: %.2f)\n", current.x, current.y, current.a);
    double dx = robot->goal.x - current.x;
    double dy = robot->goal.y - current.y;
    double target_angle = atan2(dy, dx);
    double angle_error = target_angle - current.a;

    // Normalizar error entre -pi +pi
    while (angle_error > M_PI)
      angle_error -= 2 * M_PI;
    while (angle_error < -M_PI)
      angle_error += 2 * M_PI;

    robot->pos->SetXSpeed(velocidad);
    robot->pos->SetTurnSpeed(angle_error);
  }

  return 0;
}
