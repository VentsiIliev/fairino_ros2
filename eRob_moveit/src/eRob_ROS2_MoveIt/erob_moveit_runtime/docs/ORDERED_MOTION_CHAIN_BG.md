# Ordered Motion Chain в ROS2 runtime

## 1. Цел и обхват

Този документ описва ROS2 страната на `ordered_motion_chain`: публичния REST API, runtime gateway слоя, MoveIt backend-а, планиращия worker, scheduler bridge-а, изпълнението към `FollowJointTrajectory` контролера, статусите, blending, readiness бариерите, trajectory concatenation и prepared-chain режима.

Paint системата е използвана само като практически пример. Нейната геометрия, UI и бизнес логика не са част от ROS2 runtime API.

Документът отговаря на текущата имплементация. `execution_policy="concatenate"` е нова функционалност и трябва да се потвърди на реалния робот преди производствена употреба.

---

## 2. Ментален модел

Една ordered chain заявка съдържа логически сегменти:

```text
REST JSON
  → валидация и нормализация
  → последователно планиране от очакваното крайно състояние
  → публикуване на готовите trajectory сегменти в queue
  → изпълнение в оригиналния логически ред
  → FollowJointTrajectory controller
```

Планирането и изпълнението вървят паралелно. Докато роботът изпълнява сегмент N, planning worker-ът може да планира N+1, N+2 и т.н.

Има четири различни механизма, които лесно могат да бъдат объркани като „групиране“:

| Понятие | Предназначение | Променя геометрията | Един controller goal |
|---|---|---:|---:|
| Scheduler `MotionGroup` | Вътрешна класификация по planner и hard-stop граници | Не | Не непременно |
| `readiness_group` | Не позволява първият член да започне, преди всички членове да са планирани | Не | Не |
| `execution_group` + `concatenate` | Съединява планираните joint trajectories | Не | Да |
| `blendR > 0` | Пространствено изглажда поредица LIN/PTP около междинните цели | Да | Да |

Никога не приемайте, че тези механизми са взаимозаменяеми.

---

## 3. Публичен REST API

### 3.1 Незабавно pipelined изпълнение

```http
POST /execute/ordered_motion_chain
Content-Type: application/json
```

Обща заявка:

```json
{
  "segments": [],
  "tool": 1,
  "user": 0,
  "blocking": true,
  "trajectory_optimizer": "RUCKIG"
}
```

Параметри:

| Поле | Тип | Default | Поведение |
|---|---|---:|---|
| `segments` | array | задължително | Непразен списък в точния ред за изпълнение. |
| `tool` | integer | `0` | Избира TCP/tool transform за IK и Cartesian planning. |
| `user` | integer | `0` | Избира workobject/user frame. Cartesian позициите се преобразуват към base frame. |
| `blocking` | boolean | `true` | Полето се нормализира и се препраща до backend-а, но текущата ordered-chain pipeline остава синхронна и HTTP handler-ът чака крайния result code. То не прави заявката fire-and-forget и не променя геометрията. |
| `trajectory_optimizer` | `TOTG`, `RUCKIG` или `null` | runtime default | Избира time parameterization/optimization. REST parser-ът отхвърля други стойности. |

### 3.2 Prepared chain

```http
POST /execute/ordered_motion_chain/prepare
POST /execute/ordered_motion_chain/prepared/{plan_id}/execute
GET  /execute/ordered_motion_chain/prepared/{plan_id}
DELETE /execute/ordered_motion_chain/prepared/{plan_id}
```

Prepare заявката има всички горни полета и допълнително:

| Поле | Тип | Поведение |
|---|---|---|
| `start_position` | 6 числа | Задължителна бъдеща начална Cartesian поза `[x,y,z,rx,ry,rz]`. |
| `allow_servo_during_prepare` | boolean | Ако е `true`, неоторизираният prepared plan не блокира Servo проверката. Самото изпълнение пак не може да започне при работещ Servo. |

Prepare стартира planning worker, но той чака authorization event преди физическо изпълнение. `execute` проверява дали текущата поза още съвпада с `start_position`. Стандартните tolerances са 2 mm и 2 deg, освен ако robot-specific config не ги промени.

Prepared plan състояния:

```text
planning → executing → completed
                   ↘ failed
planning → discarding → discarded
```

Не може да има повече от един активен prepared/executing plan. Неоторизиран orphan plan се изчиства след `PREPARED_CHAIN_ORPHAN_TTL_S` (default 30 s, минимум 5 s), когато роботът е idle.

### 3.3 Статус и stop

```http
GET /execute/ordered_motion_chain/status
```

Общият runtime stop механизъм задава `_ordered_motion_chain_stop_requested`. Planning worker-ът се прекратява, а execution loop-ът спира преди следващия сегмент. Текущ controller goal се отменя от общата stop логика.

---

## 4. Обща схема на сегмент

```json
{
  "type": "linear",
  "label": "human-readable name",
  "vel": 30.0,
  "acc": 30.0,
  "blendR": 0.0,
  "protected": false,
  "limit_profile": "profile_name",
  "readiness_group": "group_name",
  "execution_group": "group_name",
  "execution_policy": "concatenate"
}
```

### 4.1 Общи параметри

| Поле | Поведение |
|---|---|
| `type` | `linear`, `ptp`, `path` или `unwind_joint6`. REST parser-ът приема и legacy alias `kind`, когато `type` липсва, но новите клиенти трябва да използват `type`. Ако са подадени и двете, `type` има приоритет. |
| `label` | Име в логове и status. Default е `segment_N`. Не управлява motion behavior. |
| `vel` | Числова процентна команда, преобразувана до scale 0.0–1.0. REST parser-ът не налага диапазон; planning слоят clamp-ва ефективната стойност: ≤0 → 0.0, ≥100 → 1.0. Ако липсва, се използва runtime default (обикновено 30%). |
| `acc` | Аналогично на `vel`, но за acceleration scaling; REST не налага диапазон, а planning scale се clamp-ва до `[0,1]`. |
| `blendR` | Радиус в mm за spatial blending. `0` означава hard boundary. За LIN/PTP отрицателна стойност се отхвърля от REST; за PATH/UNWIND полето в момента се игнорира при REST нормализацията. Само LIN/PTP участват в blend group. |
| `protected` | Маркер в execution status. Клиент като paint системата може да отложи pause/stop, докато защитеният сегмент приключи. Сам по себе си backend marker-ът не отменя хардуерен emergency stop. |
| `limit_profile` | Име на robot-specific limit profile. `config.apply_limit_profiles()` го разрешава преди planning. Липсващ/невалиден profile е hard request error. Позволени са букви, цифри, `_`, `-`. |
| `readiness_group` | Група за planning barrier. Членовете трябва да са contiguous и със същото непразно безопасно име. Преди първия член executor-ът изчаква всички членове да са планирани. |
| `execution_group` | Име на максимален contiguous блок за общ controller execution. Няма ефект без `execution_policy="concatenate"`. Повторение на същото име по-късно се третира като нов блок, а не като продължение на стария. |
| `execution_policy` | В момента единствената приета стойност е `concatenate`. Без непразен `execution_group` worker-ът я игнорира. Всички членове на реалната група трябва изрично да имат същите group и policy. |

Имената на `readiness_group`, `execution_group` и `limit_profile` са ограничени до `[A-Za-z0-9_-]+`.

### 4.2 `linear`

```json
{
  "type": "linear",
  "position": [100, 200, 300, 180, 0, 90],
  "vel": 10,
  "acc": 5,
  "blendR": 0
}
```

`position` е задължителна 6D TCP поза. Транслацията е в mm, ориентацията е в deg. Позата се интерпретира в избрания `user` frame и се преобразува в base frame. LIN planner-ът планира Cartesian движение от предишния планиран край, не непременно от live robot state.

### 4.3 `ptp`

```json
{
  "type": "ptp",
  "position": [100, 200, 300, 180, 0, 90],
  "vel": 60,
  "acc": 40,
  "blendR": 20
}
```

PTP planner-ът избира валиден IK branch до TCP целта. Ако целта е effectively същата, сегментът може да стане `noop`. No-op се изпълнява логически без controller goal. Вътрешен no-op в blend/concatenate група е невалиден, защото не може надеждно да дефинира групова trajectory граница.

### 4.4 `path`

```json
{
  "type": "path",
  "path": [
    [100, 200, 300, 180, 0, 90],
    [110, 205, 300, 180, 0, 92]
  ],
  "vel": 100,
  "acc": 100,
  "blendR": 0
}
```

Предвиденият contract за `path` е непразен списък от 3D или 6D waypoints:

- 3D `[x,y,z]`: използва ориентацията от текущия планиран Cartesian state;
- 6D `[x,y,z,rx,ry,rz]`: използва зададената ориентация.

Planner-ът винаги prepends предишния планиран край към command path. Това осигурява state continuity.

Важно за текущата реализация: REST parser-ът проверява само дали `path` е truthy и не копира `blendR` за PATH. Строгата 3D/6D проверка се прави от typed validation adapter-а, но backend-ът при failure записва warning/status metadata и продължава по legacy pipeline. Следователно клиентът не трябва да разчита на REST rejection за waypoint с друга размерност. Подаден `blendR` при PATH в момента се игнорира, вместо да бъде отхвърлен; PATH planner-ът ефективно работи без blending. Това е известна contract-validation пролука.

Paint detach waypoint-ът в момента остава част от paint path-а и следователно наследява paint timing параметрите.

### 4.5 `unwind_joint6`

```json
{
  "type": "unwind_joint6",
  "vel": 40,
  "acc": 20,
  "queue_if_busy": true
}
```

Планира или изпълнява canonical unwind на Joint 6 според runtime config и текущия chain context. `queue_if_busy` указва поведението при зает motion runtime. `blendR` не се използва: REST parser-ът не го пренася за този тип, така че подадена стойност се игнорира. Ако unwind не е нужен, логическият сегмент завършва успешно без физическо движение.

---

## 5. Planning pipeline

### 5.1 Начално състояние

`build_ordered_initial_planning_state()` избира:

- live Cartesian/joint state при нормална chain заявка;
- explicit `start_position` и съответния IK state при prepared chain;
- tool transform според `tool`;
- optimizer според заявката/runtime default.

Всеки следващ сегмент се планира от `previous_target` и `previous_state`, получени от края на предишната планирана trajectory. Така planning worker-ът не зависи от това докъде физически е стигнал роботът.

### 5.2 Parallel planning/execution

`run_ordered_planning_and_execution()` стартира един planning thread и изпълнява consumer loop в caller thread. Planning queue запазва реда на сегментите. Ако planning fail-не, queue публикува exception и execution fail-ва затворено.

`plan_timeout_s` е максималното чакане за следващ необходим planned item. Timeout не е допустимо „прескачане“ — цялата chain се проваля.

### 5.3 Readiness barrier

Пример:

```json
[
  {"type":"linear", "label":"attach", "readiness_group":"contact_1"},
  {"type":"path", "label":"paint", "readiness_group":"contact_1"}
]
```

Consumer-ът prefetch-ва всички contiguous членове на `contact_1`, преди да изпълни `attach`. Ако paint planning отнеме 4 s, роботът чака преди attach. След release няма planning wait между членовете.

Readiness group:

- не обединява controller goals;
- не променя trajectory;
- не променя `vel`/`acc`;
- не гарантира липса на controller handoff delay;
- трябва да е contiguous; повторна поява след друга група е validation error.

### 5.4 Scheduler `MotionGroup`

Typed adapter-ът преобразува JSON сегментите в `MotionSegment` модели и `group_motion_batch()` ги групира за status/migration purposes:

- последователни LIN са compatible;
- последователни PTP са compatible;
- PATH и UNWIND винаги са самостоятелни hard-stop групи;
- `blendR <= 0` затваря групата след сегмента.

Това групиране не трябва да се използва от application кода като readiness или execution contract.

Typed validation тук не е authoritative execution gate. При validation failure backend-ът публикува diagnostic warning/metadata и продължава с legacy ordered pipeline. REST parser-ът и самите planners остават реалните hard gates за изпълнението.

---

## 6. Spatial blending с `blendR`

Blending се активира, когато LIN/PTP сегмент има `blendR > 0`. Worker-ът събира последователна група, докато срещне член с `blendR == 0` или неподдържан тип.

Поведение:

1. Всеки член се планира с deferred optimization.
2. `BlendBuilder.build()` изрязва части около junction-ите.
3. Изгражда нова joint trajectory през blend samples.
4. Валидира състоянията/колизиите.
5. Прилага общ optimizer върху групата.
6. Първият logical segment носи `type="blended"` и общата trajectory.
7. Останалите стават `blend_consumed` и не изпращат нов controller goal.

`blendR` е желан Cartesian радиус в mm, но effective radius може да бъде намален спрямо дължината на съседните движения. Малък effective radius под configured минимум е error.

Следствие: blend trajectory не минава задължително през оригиналната междинна target поза. Използвайте го за свободни преходни движения, не за точка на контакт, измерване, захващане или начало на геометрично точен process path.

---

## 7. Concatenation без промяна на геометрията

### 7.1 Contract

```json
[
  {
    "type": "linear",
    "execution_group": "contact_1",
    "execution_policy": "concatenate"
  },
  {
    "type": "path",
    "execution_group": "contact_1",
    "execution_policy": "concatenate"
  }
]
```

Всички contiguous членове със същите `execution_group` и `execution_policy="concatenate"` се планират независимо. След това `_concatenate_joint_trajectories()`:

1. Проверява еднакъв joint name/order.
2. Проверява еднаква размерност.
3. Проверява junction error ≤ 0.02 rad.
4. Запазва всички точки на първата trajectory.
5. Премахва първата точка на следващата trajectory, защото тя дублира общата граница.
6. Отмества `time_from_start` на останалите точки с продължителността на предишната trajectory.
7. Изисква строго нарастващи timestamps.

Не се интерполира нов Cartesian shortcut и не се trim-ва process path. Независимите `vel`/`acc` се използват при първоначалното планиране на всеки член.

`_plan_concatenated_group()` пренася `limit_profile` и `_joint_rate_limits_rad_s` към комбинирания physical segment. Ако членовете имат различни лимити, за всяка става се избира най-ниската положителна стойност. Така controller-side joint-rate guard проверява крайния общ goal с най-строгия профил; в paint случая по-ниският Joint 6 лимит от `paint_contact` важи за цялата attach + paint trajectory. Това може безопасно да разтегли и attach частта във времето, без да променя joint positions или paint geometry.

Първият logical member става internal `type="concatenated"`. Следващите стават `concatenate_consumed`. Цялата група се изпраща като един controller goal.

### 7.2 Какво concatenation не прави

- Не е spatial blend.
- Не гарантира non-zero velocity през остър ъгъл.
- Не премахва zero-velocity boundary, ако planner-ите са я създали.
- Не променя process geometry.
- Не позволява no-op member.
- Не се активира само защото един сегмент казва „concatenate with next“; поне два contiguous сегмента трябва изрично да са в една група.

### 7.3 Readiness + concatenation

За critical transition обичайно се задават и двете:

```json
{
  "readiness_group": "contact_1",
  "execution_group": "contact_1",
  "execution_policy": "concatenate"
}
```

Concatenation по конструкция изисква всички членове да са планирани преди изграждане на общата trajectory. `readiness_group` остава полезна като изричен application-level safety contract и като status/log семантика.

---

## 8. Controller execution

За обикновен LIN/PTP/PATH сегмент executor-ът:

1. Чака planned item.
2. Проверява stop request.
3. Публикува `EXECUTING` status.
4. Проверява live start-state match, ако е включено.
5. Подготвя controller trajectory/goal.
6. Може да добави кратък start ramp за меко потегляне.
7. Изпраща `FollowJointTrajectory` goal.
8. Чака action result с timeout, изчислен от trajectory duration.
9. Проверява live end-state match.
10. Публикува `DONE` или `FAILED`.

При `blended` и `concatenated` физическото изпълнение е само върху първия logical index. `*_consumed` членовете обновяват логически status без controller call.

`OrderedTrajectoryTiming` съдържа:

| Поле | Значение |
|---|---|
| `duration_s` | Timestamp на последната trajectory point. |
| `controller_goal_tolerance_s` | Допустимото controller goal-time отклонение. |
| `wait_timeout_s` | Backend timeout за action completion. |

Start-state matching default-ите са:

```text
EXECUTOR_ORDERED_START_MATCH_ENABLED = true
EXECUTOR_ORDERED_START_MATCH_TOL_RAD = 0.02
EXECUTOR_ORDERED_START_MATCH_TIMEOUT_S = 0.35
```

Това открива разминаване между планираната начална конфигурация и реалния робот. При отделни controller goals добавя latency; concatenation го премахва на вътрешната групова граница.

---

## 9. Status модел

Основни chain полета:

| Поле | Значение |
|---|---|
| `active` | Chain е в planning/execution. |
| `phase` | `starting`, `executing`, `segment_completed`, `segment_failed`, `completed`, `failed`, `stopped`, `rejected`, `error`. |
| `total_segments` | Брой логически request сегменти, не controller goals. |
| `current_segment_index` | Zero-based индекс. |
| `current_segment_number` | One-based номер за UI/log. |
| `current_segment_label/type` | Internal planned type може да бъде `blended`, `concatenated`, `blend_consumed`, `concatenate_consumed`. |
| `current_segment_protected` | Копие на protected marker-а. |
| `planned_segments_count` | Всички planned, още наблюдавани items. |
| `preplanned_ready_count` | Planned items след текущия consumer index. |
| `executed_segments_count` | Завършени логически сегменти. |
| `next_preplanned_*` | Най-близък готов сегмент. |
| `last_planned_*` | Най-далечният публикуван planned segment. |
| `result` | `0` при успех или runtime/controller error code. |

Scheduler segment state:

```text
PENDING → READY → EXECUTING → DONE
                         ↘ FAILED
```

Group state се извежда от member states. Ако който и да е member е FAILED, group е FAILED; ако всички са DONE, group е DONE.

---

## 10. Функции и методи: REST и gateway

### `parse_execute_ordered_motion_chain_request(data)`

Валидира request shape, segment types, LIN/PTP position, optimizer, LIN/PTP blend radius и safe names. За PATH проверява само непразна стойност; за PATH/UNWIND не пренася `blendR`. Създава нов normalized dictionary. Ново поле трябва изрично да бъде копирано тук — иначе ще бъде загубено между клиента и backend-а.

### `parse_prepare_ordered_motion_chain_request(data)`

Извиква основния parser и добавя `start_position`, `allow_servo_during_prepare`, `blocking=true`.

### `RuntimeApiHandlers.execute_ordered_motion_chain(data)`

Парсва заявката, проверява motion stack readiness и делегира към gateway. Превръща result code в HTTP response.

### `RuntimeApiHandlers.prepare_ordered_motion_chain(data)`

Парсва prepared заявката и създава background prepared record.

### `LocalRuntimeGateway.execute_ordered_motion_chain(...)`

Тънък adapter; препраща `segments`, `tool`, `user`, `blocking`, `trajectory_optimizer` към robot backend без motion логика.

### Prepared gateway методи

- `prepare_ordered_motion_chain(...)`: стартира planning със забранено физическо изпълнение до authorization;
- `execute_prepared_ordered_motion_chain(plan_id)`: проверява start pose и разрешава изпълнението;
- `prepared_ordered_motion_chain_status(plan_id)`: връща record + pipeline status;
- `discard_prepared_ordered_motion_chain(plan_id)`: fail-closed cancellation преди execution.

---

## 11. Функции и методи: backend и pipeline

### `MoveItRobotBackend.execute_ordered_motion_chain(...)`

Главна входна точка. Параметри:

- `segments`: normalized dictionaries;
- `tool`, `user`: TCP/workobject;
- `blocking`: request execution mode;
- `trajectory_optimizer`: requested optimizer;
- `start_position`: optional explicit start за prepared mode;
- `execution_authorized`: optional `Event`, който блокира execution, но не planning.

Методът reset-ва стар motion error, прилага limit profiles, изгражда typed validation/status metadata, проверява drive/hardware readiness и стартира pipelined implementation.

### `_execute_ordered_motion_chain_pipelined(...)`

Композиционен root на ordered subsystem. Създава:

- planning context и tool transform;
- initial planning state;
- scheduler runtime/bridge;
- blend builder;
- segment planner callback;
- planning worker factory;
- execution hook bundle;
- pipeline runner.

### `_set_ordered_motion_chain_status(**updates)`

Thread-safe-by-replacement status update чрез `normalize_ordered_chain_status()`. Добавя `updated_at`.

### Prepared backend методи

- `prepare_ordered_motion_chain(...)`: създава `plan_id`, authorization event и background future;
- `execute_prepared_ordered_motion_chain(...)`: не допуска активен motion/Servo и валидира start pose;
- `has_active_prepared_ordered_motion_chain()`: блокира несъвместим Servo и чисти orphan records;
- `discard_prepared_ordered_motion_chain(...)`: спира planning преди authorization;
- `get_prepared_ordered_motion_chain(...)`: връща record status;
- `_prepared_status(record)`: JSON-safe кратък изглед.

---

## 12. Функции и методи: planning

### `parse_ordered_segment_parameters(...)`

Извлича `type`, `label`, `blendR`, `vel`, `acc`, `protected` и пресмята scales. `_scale_from_percent()` clamp-ва до `[0,1]`.

### `build_ordered_segment_planner_callback(hooks)`

Затваря backend dependencies в callback с legacy signature за worker-а.

### `plan_ordered_segment(...)`

Dispatcher към LIN, PTP, PATH или UNWIND planner. Маркира `ordered_segment_plan_start` и отхвърля неизвестен тип.

### `plan_ordered_linear_segment(...)`

Прилага workobject, планира Cartesian LIN и връща trajectory dictionary. `defer_optimization=true` се използва за spatial blend preparation.

### `plan_ordered_ptp_segment(...)`

Прави IK/PTP planning, noop detection и optional optimization. Връща native/IK/validation timing diagnostics.

### `plan_ordered_path_segment(...)`

Преобразува 3D/6D waypoints, prepends текущия planned start и изгражда follow-path trajectory. Не поддържа `blendR`.

### `execute_ordered_planning_worker(...)`

Основен sequential planner loop. Приоритетът е:

1. `execution_group + concatenate`;
2. `blendR > 0`;
3. обикновен единичен segment.

След всеки planned item обновява `previous_target/state` и го публикува в bridge queue.

### `_plan_blend_group(...)`

Събира blend members, планира deferred trajectories, изгражда spatial blend, оптимизира общата trajectory и публикува `blend_consumed` placeholders.

### `_plan_concatenated_group(...)`

Събира contiguous members със същия execution group/policy, планира ги независимо, извиква concatenation helper и публикува `concatenate_consumed` placeholders.

### `_concatenate_joint_trajectories(planned_group)`

Валидира joint continuity, премахва duplicate boundary и измества timestamps. Не извиква Cartesian interpolation и не променя positions.

### `_duration_ns()` / `_set_duration_ns()`

Преобразуват ROS Duration между `(sec,nanosec)` и integer nanoseconds, за да няма floating-point timestamp drift.

---

## 13. Функции и методи: scheduler bridge

### `OrderedPlannedQueue`

- `put_planned(index, segment)`: enqueue ready item;
- `put_done()`: sentinel за нормален край;
- `put_error(exc)`: sentinel за planning failure;
- `get(timeout)`: blocking dequeue.

### `OrderedChainObservation`

Thread-safe map на planned, но още неконсумирани logical indexes:

- `mark_planned()` добавя;
- `mark_consumed()` премахва;
- `preplanned_snapshot()` генерира status counters.

### `OrderedSchedulerBridge`

- `publish_ready_segment()`: state `READY` + queue + observation;
- `wait_for_planned(expected_index, timeout)`: налага строг ред; различен index е error;
- `consume_planned()`: премахва observation entry;
- `mark_executing()` / `mark_finished()`: scheduler states;
- `publish_done()` / `publish_error()`: terminal planning signal.

### `_validate_readiness_groups(segments)`

Отхвърля readiness group, която се появява повторно след прекъсване. Това предотвратява неясни/неограничени lookahead зависимости.

### `run_ordered_planning_and_execution(...)`

Стартира planning executor, optional чака authorization и извиква sequence consumer. В `finally` спира planning thread.

---

## 14. Функции и методи: execution

### `execute_ordered_planned_sequence(...)`

Consumer loop. Prefetch-ва readiness group, проверява stop flag, consume-ва status entry и извиква planned-segment executor в оригиналния logical order.

### `build_ordered_execution_hook_bundle(...)`

Събира scheduler/status/controller callback dependencies без global coupling.

### `build_ordered_planned_segment_executor(...)`

Създава callable с runtime config: timeout, error codes, default speed/acc и unwind поведение.

### `execute_ordered_planned_segment(...)`

Dispatch по internal planned type:

- физическа trajectory: `linear`, `ptp`, `path`, `blended`, `linked_lin`, `concatenated`;
- logical-only: `blend_consumed`, `concatenate_consumed`;
- специално: `unwind_joint6`.

### `execute_ordered_timed_trajectory(...)`

Проверява start state, изпраща trajectory, чака controller result и проверява end state. При failure връща configured motion error.

### `ordered_trajectory_timing(...)`

Изчислява trajectory duration, controller tolerance и backend timeout.

### `wait_ordered_trajectory_point_match(...)`

Poll-ва live joints до tolerance или timeout. Използва canonical angular difference и логва най-лошата става.

### `execute_ordered_unwind_trajectories(...)`

Изпраща една или повече unwind trajectories последователно и прекратява при първа грешка.

### `finalize_ordered_unwind_result(...)`

Проверява explicit unwind postcondition и запазва failure timestamp/result, за да не се повтори автоматично опасен unwind.

### `start_ordered_segment_execution()` / `finish_ordered_segment_execution()`

Публикуват scheduler state, compatibility status и timing events около всеки logical segment.

### Помощни класове и функции в ordered subsystem

Следните symbols не променят самостоятелно motion contract-а, но са част от пълния вътрешен поток:

| Symbol | Роля |
|---|---|
| `OrderedSegmentPlannerHooks` | Immutable bundle с planner зависимости: node, config, LIN/PTP/PATH/UNWIND callbacks и timing callback. |
| `OrderedPlanningWorkerHooks` | Зависимостите на worker-а за planning, optimization, state reconstruction, publish и logging. |
| `build_ordered_planning_worker_factory(...)` | Връща factory/callable, който свързва `stop_planning` event с worker loop-а. |
| `OrderedPipelineRunnerConfig` | Timeout, suppression и stopped result настройки за pipeline runner-а. |
| `OrderedSchedulerRuntime` | Държи scheduler, batch/group metadata, bridge, queue и observation за една chain. |
| `build_ordered_scheduler_runtime(...)` | Валидира/адаптира mappings, създава scheduler groups и bridge callbacks. |
| `OrderedTrajectoryTiming` | Изчислени duration/tolerance/timeout стойности за controller execution. |
| `OrderedSegmentExecutionHooks` | Status, stop, scheduler и dispatch callbacks за logical segment execution. |
| `OrderedControllerExecutionHooks` | Controller send/wait, state read и error callbacks. |
| `OrderedUnwindFinalizationHooks` | Postcondition и error-state callbacks за unwind. |
| `OrderedStateMatchHooks` | Joint-state polling, tolerance и canonical-angle callbacks. |
| `OrderedPlannedSequenceHooks` | Consumer-loop dependencies: wait, consume, execute, stop и status. |
| `OrderedExecutionHookBundle` | Общ immutable bundle на execution hook групите. |
| `OrderedPlannedSegmentExecutorConfig` | Error codes, timeouts, default vel/acc и unwind runtime settings. |
| `_duration_to_seconds()` / `_set_duration_from_seconds()` | ROS Duration ↔ floating seconds за execution timing/stretching. |
| `_scale_optional_sequence(values, scale)` | Скалира velocity/acceleration arrays, ако присъстват. |
| `_configured_joint_rate_limits()` | Чете валидните joint-rate limits от runtime config. |
| `_ordered_blend_rate_guard_enabled()` | Проверява feature/config switch за joint-rate guard. |
| `_maybe_stretch_ordered_blend_joint_rates(...)` | Удължава времето на цяла physical trajectory, ако измереният joint rate надвишава metadata limits; не променя positions. |
| `ordered_trajectory_point_match_error(...)` | Изчислява максималната canonical joint грешка и най-лошата става. |

Typed mapping/scheduler слой:

| Symbol | Роля |
|---|---|
| `OrderedMotionBatchValidation` / `OrderedMotionBatchValidationFailure` | Успешен или неуспешен typed validation резултат с log/timing serialization. |
| `_as_float`, `_as_pose6`, `_as_waypoints` | Строги typed conversion helpers. |
| `ordered_segment_from_mapping(...)` | Един dictionary → `MotionSegment`. |
| `ordered_segments_from_mappings(...)` | Последователност mappings → tuple typed segments. |
| `ordered_motion_batch_from_mappings(...)` | Създава `MotionBatch` с tool/user/blocking metadata. |
| `validate_ordered_motion_batch_from_mappings(...)` | Non-throwing validation wrapper, използван за diagnostics/migration. |
| `planner_name_for_segments(...)` | Определя planner label за група. |
| `segments_are_group_compatible(left, right)` | Проверява дали два typed segments могат да са в един scheduler group. |
| `has_hard_stop_after(segment)` | Определя typed hard-stop границата. |
| `MotionScheduler.group_batch()` / `group_motion_batch()` | Изграждат `MotionGroup` tuple и member indexes. |

Status adapter слой:

| Symbol group | Роля |
|---|---|
| `ordered_chain_group_status`, `ordered_chain_initial_group_states` | JSON-safe group metadata и начални group states. |
| `_group_state_from_segment_states`, `update_ordered_chain_group_states_from_segments`, `ordered_chain_group_state_status` | Извеждат group state от member states. |
| `ordered_chain_initial_segment_states`, `ordered_chain_initial_segment_states_from_mappings`, `ordered_chain_segment_state_status` | Създават и сериализират logical segment states. |
| `normalize_ordered_chain_status` | Допълва липсващи полета и поддържа стабилна status schema. |
| `ordered_chain_starting_status`, `ordered_chain_executing_status`, `ordered_chain_segment_finished_status` | Status transitions при start, execution и logical completion. |
| `ordered_chain_stopped_status`, `ordered_chain_terminal_status` | Terminal status при stop/success/failure. |
| `ordered_chain_preplanned_snapshot` | Броячи и labels за ready-but-not-consumed сегментите. |

---

## 15. Paint пример: soft attach + exact paint path

Paint application изгражда приблизително:

```python
group = "paint_contact_1"

attach = {
    "type": "linear",
    "label": "paint_attach_1",
    "position": first_contact_pose,
    "vel": contact_staging.attach_vel_percent,   # напр. 10%
    "acc": contact_staging.attach_acc_percent,   # напр. 5%
    "blendR": 0.0,
    "protected": True,
    "readiness_group": group,
    "execution_group": group,
    "execution_policy": "concatenate",
}

paint = {
    "type": "path",
    "label": "paint_contact_1:Workpiece",
    "path": command_path,
    "vel": job["vel"],                          # напр. 100%
    "acc": job["acc"],                          # напр. 100%
    "protected": True,
    "limit_profile": "paint_contact",
    "readiness_group": group,
    "execution_group": group,
    "execution_policy": "concatenate",
}
```

Резултат:

```text
pickup/staging се изпълнява, докато attach + paint се планират
→ ако planning изостане, чакането е в safe staging pose
→ attach се планира с 10/5
→ paint се планира с job 100/100
→ joint trajectories се съединяват без Cartesian trim
→ един controller goal изпълнява attach + paint
```

Защо `blendR` остава 0:

- контактната поза трябва да се достигне точно;
- spatial blend би отрязал ъгъла между normal attach и tangential paint motion;
- concatenation премахва controller handoff паузата, без да променя paint waypoints.

Attach settings идват от Paint Process Settings → Motion Speeds → Paint Contact и се сериализират като:

```json
"contact_staging": {
  "attach_vel_percent": 10.0,
  "attach_acc_percent": 5.0
}
```

ROS2 runtime не познава тези UI полета; той вижда само вече попълнените `vel` и `acc` в attach сегмента.

---

## 16. Логове и диагностика

Ключови events:

| Event | Значение |
|---|---|
| `backend_received` | Backend е получил chain заявката. |
| `ordered_planning_worker_start/done/error` | Жизнен цикъл на planner thread. |
| `ordered_segment_plan_start/done` | Planning timing за logical segment. |
| `ordered_segment_queued` | Planned item е публикуван към consumer-а. |
| `ordered_plan_wait_start/ready` | Consumer чака конкретен index. |
| `ordered_readiness_group_ready` | Всички членове на readiness group са ready преди първия execute. |
| `ordered_segment_execute_start/done` | Logical execution lifecycle. |
| `ordered_state_match_start/done` | Live joint start/end verification. |
| `ordered_controller_handoff_start` | Начало на controller preparation. |
| `controller_prepare_start/done` | Mutation и goal construction latency. |
| `goal_send/goal_accepted` | Action handoff към контролера. |
| `ordered_wait_execution_start/done` | Blocking wait върху action result. |

При правилно concatenated paint изпълнение очаквайте:

```text
ordered_segment_plan_done ... paint_attach_1
ordered_segment_plan_done ... paint_contact_1
ordered_segment_queued ... type=concatenated execution_group=paint_contact_1
ordered_segment_execute_start ... type=concatenated
goal_send ... един общ goal
ordered_segment_execute_start ... type=concatenate_consumed
```

Не трябва да има втори `goal_send` между attach и paint.

---

## 17. Validation и типични грешки

| Грешка | Причина |
|---|---|
| `Missing non-empty 'segments'` | Липсва/празен списък. |
| `unsupported type` | Непознат segment `type`. |
| `linear position must have 6 values` | LIN или PTP target не е 6D. Съобщението в parser-а казва `linear` и за PTP. |
| `path is required` | PATH няма waypoints. |
| `path cannot use blendR` | Възможно при директно вътрешно backend извикване, което е запазило полето. Публичният REST parser в момента го премахва и така не достига тази грешка. |
| `readiness_group must be contiguous` | Едно име се появява отново след прекъсване. |
| `execution_group requires at least two` | Самотен concatenate member. |
| `joint order mismatch` | Груповите trajectories не са за еднакъв controller/joint order. |
| `junction mismatch` | Краят на лявата и началото на дясната trajectory се различават над 0.02 rad. |
| `non-increasing timing` | Следваща trajectory има невалидни timestamps. |
| `start state mismatch` | Реалният робот не съвпада с планираното начало. |
| `prepared chain start mismatch` | Роботът се е преместил след prepare. |
| `drive not enabled/hardware not ready` | Safety/hardware preflight rejection. |

---

## 18. Правила за безопасна употреба

1. Използвайте `label`, който описва физическото намерение.
2. Не използвайте `blendR` около контактни или измервателни точки.
3. Използвайте readiness barrier преди навлизане в constrained region.
4. Използвайте concatenation, когато geometry трябва да остане точна, но controller handoff е недопустим.
5. Не разчитайте само на `protected`; хардуерният safety stop винаги има приоритет.
6. Проверявайте `tool` и `user`, защото грешен frame променя всички Cartesian цели.
7. Не променяйте runtime segment schema без съответна промяна в REST parser-а — непознатите полета се губят при нормализация.
8. След промяна rebuild-нете правилния workspace, source-нете правилния `install/setup.bash` и рестартирайте процеса.
9. Валидирайте нова chain първо с fake hardware и после на ниски limits.
10. За concatenation потвърдете от логовете, че има един controller `goal_send` за цялата execution group.

---

## 19. Карта на основните файлове

```text
scripts/rest/api_support.py
    REST schema и нормализация

scripts/runtime_api/handlers.py
    HTTP-facing orchestration

scripts/runtime_gateway/local.py
    API → backend adapter

scripts/backend/moveit_robot_backend.py
    lifecycle, safety preflight, prepared chain, pipeline composition

scripts/motion/planning/ordered_*_planner.py
    LIN/PTP/PATH/UNWIND planning

scripts/motion/scheduling/ordered_planning_worker.py
    lookahead planning, blend и concatenate group construction

scripts/motion/scheduling/ordered_scheduler_bridge.py
    queue, observation и scheduler states

scripts/motion/scheduling/ordered_pipeline_runner.py
    planning thread + execution consumer orchestration

scripts/motion/execution/ordered_execution.py
    readiness barrier, controller execution и logical consumed segments

scripts/motion/blending/blend_builder.py
    spatial LIN/PTP blending

scripts/motion/scheduling/status_adapter.py
    JSON-safe status schema
```
