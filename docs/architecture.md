# COMRaDE Architecture

## Goals

Библиотека должна поддерживать два связанных, но разных контура расчета:

1. Контур контактной физики сцены:
   `Bullet` отвечает за гравитацию, столкновения, broadphase/narrowphase, контактные пары и внешние воздействия.
2. Контур динамики робота:
   собственная многотельная модель манипулятора интегрируется независимо через `boost::odeint`.

Ключевая идея: робот не должен быть "еще одним rigid body" в Bullet. Для робота `Bullet` используется как сервис обнаружения контактов и оценки внешних сил, а состояние сочленений и звеньев обновляется в собственной системе уравнений движения.

## Domain Model

На уровне сцены все сущности делятся на 4 группы:

1. `VisualEntity`
   Только визуализация, не участвует в коллизиях и динамике.
2. `StaticCollisionEntity`
   Статический объект сцены. Имеет collision shape в `Bullet`, но не интегрируется по времени.
3. `DynamicRigidEntity`
   Обычное твердое тело `Bullet`: масса, инерция, гравитация, столкновения.
4. `ArticulatedSystemEntity`
   Многотельная система со своей математической моделью. Для манипулятора состояние определяется обобщенными координатами `q`, скоростями `dq` и внутренней моделью динамики.

## Proposed Module Layout

```text
include/
  comrade/
    core/
      Types.h
      Id.h
      Status.h
      TimeStep.h
    scene/
      Scene.h
      SceneGraph.h
      Entity.h
      EntityVisitor.h
      EntityFactory.h
    scene/components/
      TransformComponent.h
      VisualComponent.h
      CollisionComponent.h
      PhysicsComponent.h
      RobotModelComponent.h
    physics/
      IPhysicsWorld.h
      BulletWorld.h
      BulletConverters.h
      CollisionEvent.h
      ContactInfo.h
      ForceTorque.h
    dynamics/
      IDynamicSystem.h
      ODEState.h
      IntegratorConfig.h
      RobotDynamicsModel.h
      RobotDynamicsSolver.h
      ExternalWrenchAccumulator.h
    robotics/
      RobotModel.h
      Link.h
      Joint.h
      JointState.h
      KinematicsSolver.h
      DynamicsParameters.h
      URDFLoader.h
    simulation/
      SimulationEngine.h
      SimulationContext.h
      SimulationStepResult.h
      Synchronizer.h
    rendering/
      IRenderBridge.h
      OsgSceneBridge.h
    io/
      SceneLoader.h
      SceneSerializer.h

src/
  core/
  scene/
  physics/
  dynamics/
  robotics/
  simulation/
  rendering/
  io/
```

## Layer Responsibilities

### `core`

Базовые типы, ошибки, идентификаторы, конфигурация шага интегрирования, общие перечисления.

### `scene`

Семантика сцены и иерархия сущностей.

Слой не должен зависеть от конкретной физики или визуализации сильнее, чем через абстракции и компоненты.

Основные обязанности:

- хранение дерева сущностей;
- владение компонентами;
- маршрутизация обновления;
- фабрики объектов сцены.

### `physics`

Адаптер к `Bullet`.

Основные обязанности:

- создание и владение `btDiscreteDynamicsWorld`;
- регистрация статических и динамических collision objects;
- шаг контактной физики;
- извлечение контактных данных;
- преобразование результатов столкновений в внешние силы/моменты.

Важно: слой `physics` не должен решать уравнения движения робота по сочленениям.

### `dynamics`

Чистая математическая динамика многотельных систем.

Основные обязанности:

- хранение вектора состояния `x = [q, dq]`;
- вычисление правой части `dx/dt = f(x, t, tau, w_ext)`;
- интегрирование через `boost::odeint`;
- накопление внешних нагрузок `w_ext` от контактов и гравитации;
- расчет внутренних сил, обобщенных моментов и реакций.

### `robotics`

Описание робота как механической системы.

Основные обязанности:

- модель звеньев и сочленений;
- кинематика;
- инерционные параметры;
- ограничения суставов;
- загрузка из URDF/собственного формата;
- отображение между физическими контактами и звеньями модели.

### `simulation`

Оркестратор всей симуляции.

Основные обязанности:

- согласованный шаг `Bullet` и `odeint`;
- синхронизация transform/state между подсистемами;
- политика substep-ов;
- публикация результатов кадра.

### `rendering`

Мост к OSG или другому движку визуализации.

Этот слой получает уже готовые трансформации и геометрию, но не должен содержать физическую логику.

## Main Interfaces

### Scene Entity

```cpp
class Entity {
public:
    virtual ~Entity() = default;
    virtual EntityKind kind() const noexcept = 0;
    virtual void onAttached(Scene& scene) {}
    virtual void preStep(const SimulationContext& ctx) {}
    virtual void postStep(const SimulationContext& ctx) {}
};
```

### Physics World

```cpp
class IPhysicsWorld {
public:
    virtual ~IPhysicsWorld() = default;
    virtual void addStatic(const CollisionComponent&) = 0;
    virtual void addDynamic(const PhysicsComponent&) = 0;
    virtual void updateKinematicBody(EntityId, const Transform3&) = 0;
    virtual void step(double dt) = 0;
    virtual std::vector<ContactInfo> contacts() const = 0;
};
```

### Dynamic System

```cpp
class IDynamicSystem {
public:
    virtual ~IDynamicSystem() = default;
    virtual void setControl(const Eigen::VectorXd& tau) = 0;
    virtual void setExternalWrenches(const std::vector<ForceTorque>& wrenches) = 0;
    virtual void integrate(double dt) = 0;
    virtual const ODEState& state() const = 0;
};
```

## Robot Representation

Для манипулятора лучше разделить 3 представления одного и того же робота:

1. Геометрическое представление
   Нужно для визуализации.
2. Контактное представление
   Набор `Bullet`-форм на звеньях для детекции столкновений.
3. Динамическое представление
   Математическая модель с массами, инерциями, `q`, `dq`, `tau`.

Это позволит не смешивать визуальную mesh-модель, collision-модель и динамическую модель.

## Contact Pipeline For Robots

Рекомендуемый поток данных для робота:

1. После интегрирования состояния робота вычисляются позы всех звеньев.
2. Эти позы передаются в `Bullet` как kinematic collision objects звеньев.
3. `Bullet` выполняет поиск столкновений со статикой и динамическими объектами.
4. Контакты собираются как `ContactInfo`:
   идентификатор звена, точка контакта, нормаль, глубина проникновения, импульс/оценка силы.
5. Контакты преобразуются в внешние wrench-нагрузки на звенья.
6. Через якобиан или mapping link-to-generalized-coordinates нагрузки переводятся в обобщенные силы `Q_ext`.
7. На следующем шаге `RobotDynamicsSolver` интегрирует систему уже с учетом `Q_ext`.

Это важное разделение: `Bullet` дает контактную информацию, но не владеет состоянием шарниров робота.

## Recommended Simulation Loop

```text
for each frame:
  1. apply controls to robots
  2. integrate articulated dynamics by dt_sub
  3. update robot link poses from q
  4. push robot collision geometry into Bullet as kinematic bodies
  5. step Bullet for dynamic rigid entities
  6. collect contacts:
       - rigid-rigid
       - robot-static
       - robot-dynamic
  7. convert robot contacts to external generalized forces
  8. synchronize transforms to rendering scene
```

Если нужна более устойчивая связь контактной физики и динамики робота, шаги 2-7 можно выполнять в нескольких внутренних итерациях на одном кадре.

## Separation Of Object Categories

Для типов объектов полезно ввести не наследование "по каждому случаю", а комбинацию ролей:

- `Renderable`
- `Collidable`
- `RigidBody`
- `ArticulatedBody`
- `Sensor`

Тогда 4 пользовательских типа сцены будут просто профилями конфигурации:

- только визуальный: `Renderable`
- статичный: `Renderable + Collidable`
- динамический: `Renderable + Collidable + RigidBody`
- многотельный робот: `Renderable + Collidable + ArticulatedBody`

Такой подход уменьшает число жестко прошитых классов и упрощает расширение библиотеки.

## Suggested Concrete Classes

### Scene side

- `Scene`
- `SceneGraph`
- `EntityRegistry`
- `EntityFactory`
- `TransformComponent`
- `VisualComponent`
- `CollisionComponent`
- `RigidBodyComponent`
- `ArticulatedBodyComponent`

### Bullet side

- `BulletWorld`
- `BulletBodyHandle`
- `BulletCollisionShapeCache`
- `BulletContactCollector`
- `BulletDebugDrawer`

### Robot dynamics side

- `RobotModel`
- `RobotState`
- `RobotKinematics`
- `RobotDynamicsModel`
- `RobotDynamicsSolver`
- `ExternalWrenchAccumulator`
- `JointLimitModel`

### Simulation side

- `SimulationEngine`
- `SimulationConfig`
- `StepScheduler`
- `PhysicsSynchronizer`
- `RobotContactMapper`

## Data Contracts

Минимальные структуры обмена между подсистемами:

```cpp
struct Transform3 {
    Eigen::Vector3d position;
    Eigen::Quaterniond rotation;
};

struct ContactInfo {
    EntityId self;
    EntityId other;
    std::string selfLinkName;
    Eigen::Vector3d pointWorld;
    Eigen::Vector3d normalWorld;
    double penetrationDepth;
    double normalImpulse;
};

struct ForceTorque {
    std::string linkName;
    Eigen::Vector3d forceWorld;
    Eigen::Vector3d torqueWorld;
    Eigen::Vector3d applicationPointWorld;
};

struct RobotState {
    Eigen::VectorXd q;
    Eigen::VectorXd dq;
};
```

## Important Design Decisions

### 1. Keep Bullet behind an interface

Не стоит распространять `btRigidBody*`, `btCollisionShape*` и `btTransform` по всей библиотеке. Лучше держать их внутри адаптера `physics`.

### 2. Separate contact detection from robot dynamics

Контакт и интегрирование многотельной системы должны быть разными этапами. Это упростит отладку, замену решателя и тестирование.

### 3. Use fixed simulation timestep

Для численной устойчивости нужен фиксированный шаг, например `dt = 1e-3` или `5e-4`, а визуализация может обновляться реже.

### 4. Support kinematic collision proxies for robot links

Для каждого звена робота в `Bullet` лучше иметь collision proxy без передачи владения динамикой самому Bullet.

### 5. Build deterministic logging and replay

Полезно сразу предусмотреть логирование:

- управляющих воздействий;
- контактов;
- состояний `q`, `dq`;
- внешних сил по звеньям.

Это сильно упростит верификацию модели.

## How This Maps To Current Repository

Текущее состояние проекта уже содержит полезные заготовки:

- `EntitySystem` можно развить в слой `scene`;
- `RobotMath` можно развить в слои `robotics` и `dynamics`;
- код инициализации `Bullet` из `Scene` лучше вынести в отдельный `BulletWorld`;
- `MultiBody` стоит превратить из пустой сущности в контейнер:
  `RobotModel + RobotState + RobotDynamicsSolver + collision proxies`.

## Recommended Next Implementation Steps

1. Разделить текущий `Scene` на:
   `Scene`, `SimulationEngine`, `BulletWorld`.
2. Убрать прямую зависимость базовых `Entity` от `Bullet` и `OSG`, оставив компоненты и абстракции.
3. Переписать `MultiBody` в `ArticulatedSystemEntity`.
4. Ввести `ContactInfo`, `ForceTorque`, `RobotState`.
5. Сделать `RobotDynamicsSolver` с простым state vector на `boost::odeint`.
6. Реализовать `RobotContactMapper`: контакт звена -> wrench -> generalized force.
7. После этого подключить OSG-мост как отдельный слой синхронизации.

## Minimal MVP

Минимально жизнеспособная версия может поддерживать:

- одну сцену;
- один манипулятор;
- статические препятствия;
- динамические коробки в `Bullet`;
- контакт звено-объект;
- внешнюю силу на звено;
- интегрирование `q`, `dq` через `odeint`;
- визуализацию результата.

Если MVP заработает в такой конфигурации, дальше можно уверенно добавлять:

- несколько роботов;
- захваты;
- приводы;
- датчики;
- мягкие контакты;
- контроллеры.
