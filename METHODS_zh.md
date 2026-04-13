# 方法说明文档

本文档说明当前分支 `wip/prefuel-search-save` 上的实现方法。

当前方案的目标不是构造一个全局最优的集中式规划器，而是在课程给定的 Tileworld 框架中，构建一套：

- 可以解释
- 运行稳定
- 计算代价可控
- 适合多 agent 协作

的实用策略系统。

整体方法由以下几层组成：

- 反应式决策
- 基于短期效用的目标选择
- 基于 A* 的局部路径规划
- 工作记忆与战略记忆
- 静态空间分区
- 大地图上的 pre-fuel sector 搜站
- 大地图上的 sector 探索
- 轻量级 agent 间通信

## 1. 运行环境与默认配置

基准程序入口在 `src/tileworld/TileworldMain.java`。

默认参数在 `src/tileworld/Parameters.java` 中定义：

- 每次运行 `5000` 步
- 默认 `6` 个 agent
- 初始 fuel 为 `500`
- sensor range 为 `3`

系统支持两套公开配置。

### `config1`

- 地图大小：`50 x 50`
- `tile / hole / obstacle` 的生成均值：`0.2`
- 生成标准差：`0.05`
- 对象寿命：`100`

### `config2`

- 地图大小：`80 x 80`
- `tile / hole / obstacle` 的生成均值：`2.0`
- 生成标准差：`0.5`
- 对象寿命：`30`

两套配置共同保持：

- `6` 个 agent
- `5000` 步
- 初始 fuel `500`
- 感知范围 `3`

环境在每次 `start()` 时会：

1. 创建 object grid 和 agent grid
2. 随机生成 `6` 个 agent 的出生点
3. 随机生成一个 fuel station
4. 在之后每一步持续生成和删除 tile、hole、obstacle

环境的调度顺序是：

1. 环境更新
2. 所有 agent `sense()` 和 `communicate()`
3. 所有 agent `think()` 和 `act()`

这个顺序很重要，因为它意味着通信是“同一步广播，再同一步决策”。

## 2. 系统总体结构

当前实现主要由四部分组成。

### 2.1 环境层

文件：

- `src/tileworld/environment/TWEnvironment.java`

负责：

- 保存 object grid 和 agent grid
- 创建 agent
- 生成和删除环境对象
- 维护全局 reward
- 提供移动、阻塞、拾取和放置检查

### 2.2 基础 agent 层

文件：

- `src/tileworld/agent/TWAgent.java`

负责：

- agent 的基础移动
- pickup / putdown / refuel 动作
- fuel 变化
- 个体 score 维护
- 每一步执行入口

### 2.3 策略层

文件：

- `src/tileworld/agent/SimpleTWAgent.java`

这是当前方法的核心实现，负责：

- 通信
- 目标选择
- 探索
- fuel 管理
- 路径规划调用
- 多 agent 协作控制

### 2.4 战略记忆层

文件：

- `src/tileworld/agent/StrategicTWAgentMemory.java`

这是在原始 working memory 之上补的高层记忆，负责：

- 已知 tile 记忆
- 已知 hole 记忆
- 已知 fuel station 记忆
- sector 级的新鲜度与局部统计

## 3. Agent 的总体行为逻辑

当前 agent 是反应式的，每一步都会重新评估当前状态。

总体决策顺序是：

1. 如果当前就在 fuel station 上且油量没满，优先 `REFUEL`
2. 如果当前脚下是 hole 且手里有 tile，优先 `PUTDOWN`
3. 如果当前脚下是 tile 且还没满载，优先 `PICKUP`
4. 如果以上都不成立，再进入移动决策

这个顺序背后的理由很直接：

- `PUTDOWN` 会立刻结算 reward
- `PICKUP` 会立刻把可用资源拿到手
- `REFUEL` 会立刻恢复生存能力
- 只有当前格子没有更高价值动作时，才需要规划“往哪里走”

## 4. Strategic Memory

原始框架给的 working memory 主要是“看到什么就记到格子里”。当前方案在这之上增加了结构化的战略记忆。

### 4.1 已知目标

当前策略显式维护三类目标：

- `TILE`
- `HOLE`
- `FUEL_STATION`

每个目标都记录：

- 类型
- 坐标
- 观察时间
- 失效时间

其中：

- fuel station 一旦发现，就视为永久有效
- tile 和 hole 会随着环境对象寿命而过期

### 4.2 记忆更新流程

每次感知后，战略记忆会执行：

1. 调用底层 working memory 更新
2. 用当前视野去纠正旧记忆
3. 删除已经过期的对象
4. 记录本步刚刚看到的 tile、hole、fuel station
5. 更新当前视野覆盖到的 sector
6. 重建 sector 级的 tile/hole 统计

这里有两个非常关键的清理规则：

- 如果某个格子当前就在视野内，但原来记忆里有 tile/hole，而现在已经没有了，就删掉旧记忆
- 如果某个记忆对象的过期时间已经到了，就删除

这样做是为了防止 agent 持续追逐已经消失的旧目标。

## 5. 目标选择策略

当前方法不再是“最近就去”，而是一个两阶段的目标筛选过程。

### 5.1 第一阶段：候选过滤

目标只有在以下条件都满足时，才进入候选集：

- 目标存在于记忆中
- 目标尚未过期
- 以当前距离看，理论上来得及赶到
- fuel 预算允许
- 目标当前可见，或虽然不可见但仍然比较“新鲜”

这一层先把明显不值得追的目标删掉。

### 5.2 第二阶段：初筛评分

#### tile 评分

tile 的基础评分考虑：

- agent 到 tile 的距离
- tile 到最近 hole 的距离
- 该 tile 被观察到的时间有多旧
- 当前 agent 还空着多少携带容量
- 跨主区惩罚

含义是：

- 离自己近的 tile 更好
- 附近有 hole 的 tile 更好
- 越新鲜的 tile 越值得追
- 手里越空，继续捡 tile 的收益越大
- 深度跨区的 tile 会被惩罚

#### hole 评分

hole 的基础评分考虑：

- agent 到 hole 的距离
- hole 的观察年龄
- 当前携带了多少块 tile
- 跨主区惩罚

含义是：

- 离自己近的 hole 更好
- 越新鲜的 hole 越好
- 手里 tile 越多，hole 越值得优先送达

### 5.3 第三阶段：精排

当前实现不会对所有候选都做重计算，而是先保留一个很小的 shortlist，再做二次精排。

精排时会进一步考虑：

- A* 路径 detour
- 距离过期还有多少 slack
- 附近是否形成 cluster

对于 tile：

- detour 越小越好
- 剩余 slack 越大越好
- 周边 tile 多，说明连续作业潜力更高

对于 hole：

- detour 越小越好
- 剩余 slack 越大越好
- 如果手上有多块 tile，周边 hole 多会更有利

最终效果是：

- 不再只是单纯追最近目标
- 而是更偏向“能尽快转化成 reward 的目标”

## 6. 交付决策

一个重要问题是：手上已经有 tile 之后，应该继续捡，还是先送？

这一点由 `shouldDeliverTiles(...)` 控制。

当前规则考虑：

- 当前 hole 是否已经比较紧急
- 是否已经满载
- 是否还有合适的 tile 可以顺路再捡
- 当前 hole 的路程
- 下一个 tile 的路程
- 该 tile 附近是否存在合适的 hole

主要规则是：

- hole 快过期时，立刻送
- 手里 3 块 tile 时，立刻送
- 没有好 tile 可捡时，送
- 已经拿了 2 块 tile 后，会明显更保守

这样做是为了减少一种常见低效行为：

- agent 一直想“再贪一个 tile”
- 结果把本来已经比较稳的交付机会拖没了

## 7. Fuel 管理

fuel 管理是当前方法的核心部分之一。

### 7.1 回油判断

如果 fuel station 已知，agent 会不断判断是否应该回站。

这个判断考虑：

- 当前到 fuel station 的实际路程
- 如果手里有 tile，是否可以顺路去一个 hole 再回站
- 安全缓冲

也就是说，并不是简单的“油量低于某个固定值就回家”，而是基于当前情境计算。

### 7.2 大小图不同的保守程度

当前策略对 `80 x 80` 更保守，具体体现在：

- 更大的 `FUEL_BUFFER`
- 更大的 pre-fuel buffer
- 更大的 path safety margin
- 更短的记忆保留窗口

这是因为大地图里：

- fuel station 更难找到
- 路程更长
- 动态对象更容易在途中失效

### 7.3 目标 fuel 预算

agent 在追逐 tile 或 hole 之前，会先做 fuel 可达性检查。

如果 fuel station 已知，就会估算：

- 到目标的代价
- 目标到 fuel station 的代价
- 再加安全缓冲

为了控制计算成本，这里用了两层估计：

1. 先用便宜的乐观估计
2. 如果接近边界，再用更贵但更准的 A* 路径估计

在 fuel station 未知时，则只能退化为：

- 到目标的代价
- 加固定 pre-fuel 余量

### 7.4 大图 Pre-Fuel 搜站

在大图上，当前分支额外加入了一层专门的 pre-fuel sector 搜站。

也就是说，在 fuel station 尚未被发现之前：

- 团队不会被限制在各自主区内
- 而是会在全图范围上按 sector freshness 和 travel cost 选搜索区块
- 同时保留轻量 claim，避免所有 agent 完全重叠

这一层的目标是尽快完成第一次 fuel station 发现，减少因为搜站过慢导致的灾难性 seed。

## 8. 路径规划

当前策略使用 A* 做局部导航。

设计思路是：

- 上层决定“去哪”
- A* 负责“怎么去”

如果 A* 能找到路径：

- 当前 path 会被缓存
- agent 每一步取下一步方向

如果 A* 找不到路径：

- 会退回到一个简单的 fallback 方向选择
- 优先朝目标的 x/y 方向靠近
- 如果正方向走不通，再尝试其他方向

如果走的过程中撞到了障碍：

- 当前 path 清空
- 后续再重新规划

因此当前方案不是全局联合路径规划，而是一个：

- 低成本
- 持续重规划
- 对动态变化有一定鲁棒性

的局部导航层。

## 9. 空间组织：主区与 sector

当前团队的空间组织有两层。

### 9.1 第一层：主区

地图会按 `x` 方向被切成 6 个宏观区域，每个 agent 对应一个主区。

作用：

- 提供稳定的初始分工
- 减少重复探索
- 减少所有 agent 同时在整张图上乱跑

这里的主区是“软约束”，不是硬限制。

也就是说：

- 本区目标天然更便宜
- 但如果跨区目标足够好，仍然可以过去处理

### 9.2 第二层：sector

在主区之上，地图又被切成固定大小的 `10 x 10` sector。

每个 sector 记录：

- 边界
- 中心点
- 本地 freshness
- 团队共享 freshness
- 本地已知 tile 数
- 本地已知 hole 数
- 队友共享的 tile/hole 统计

当前 sector 主要用于大地图探索层。

## 10. 探索策略

当前探索是按配置区分的。

### 10.1 `50 x 50`

小图默认继续使用旧的 macro sweep。

这是一种稳定的蛇形覆盖方式。

之所以保留它，是因为测试发现：

- 在 `50 x 50` 上强行启用 sector exploration 会明显退化

所以当前方法有意保持：

- 小图稳定
- 大图增强

### 10.2 `80 x 80`

大图上的探索分成两个阶段。

在 fuel station 未知时：

- 先走 pre-fuel sector 搜站
- 在全图范围上按 freshness 和 travel cost 选 sector
- 目标是尽快完成第一次 fuel station 发现

在 fuel station 已知后：

- 再切回受主区约束的 sector exploration

后者的流程是：

1. 从自己主区范围内枚举候选 sector
2. 计算每个 sector 的 score
3. 选分数最高的 sector
4. 在该 sector 内构造局部 sweep 路线
5. 沿着 sector 内部 sweep 扫描

### 10.3 Sector 评分

当前 sector score 主要考虑：

- freshness
- tile opportunity
- hole opportunity
- 如果手里有 tile，则考虑交付机会
- tile-hole 配对潜力
- 到 sector center 的 travel cost
- stay bonus
- claim penalty

所以它本质上是一个 sector 级别的 utility：

- 越久没看过的 sector 越值得去
- 越有资源密度的 sector 越值得去
- 已经有队友在处理的 sector 会稍微降权

## 11. 通信设计

当前通信是刻意做成“低带宽、高价值”的，而不是全量广播。

### 11.1 Fuel Station 共享

这是当前最重要的共享信息。

只要任意一个 agent 看到了 fuel station：

- 它就会把 fuel station 坐标写入团队共享状态
- 其他 agent 会同步到自己的战略记忆里

这一点的收益非常大，因为 fuel station 是：

- 唯一的
- 稳定的
- 对全队都高价值

### 11.2 Sector Snapshot Blackboard

当前每个 agent 还会把自己本步视野覆盖到的 sector 发布成 snapshot。

每个 snapshot 包含：

- `sectorX`
- `sectorY`
- `seenAt`
- `knownTileCount`
- `knownHoleCount`

这些 snapshot 会进入一个 team-level 的共享板，再被各 agent 合并到本地记忆中。

这相当于实现了一个轻量级的 sector blackboard。

### 11.3 Shared Freshness

当前 sector state 区分了：

- `localSeenAt`
- `sharedSeenAt`

当前融合方式比较保守：

- 如果我自己从没看过某个 sector，那么队友的 freshness 信息是有价值的
- 如果我自己看过，那么仍然优先相信本地 freshness

这样做是为了避免：

- 远端共享信息过度主导决策
- 造成 agent 在局部已经清楚的情况下还被旧共享信息误导

### 11.4 Sector Claim

每个 agent 还会广播一个很轻量的 sector claim。

claim 中包含：

- agent 名字
- 当前主要处理的 sector
- 当前模式
- 报告时间

其他 agent 在给 sector 打分时，如果发现：

- 队友刚刚 claim 了同一个 sector
- 或队友正在探索邻近 sector

就会施加一个小的惩罚。

这个机制的目标不是做严格任务分配，而只是：

- 减少扎堆
- 降低局部重复劳动

## 12. 为什么通信没有做得更重

当前没有做：

- 所有 tile/hole 的全量广播
- tile 级 claim
- hole 级 claim
- 全局任务拍卖
- 异构角色协议

原因是这些方案虽然更强，但代价也明显更大：

- 信息噪声更多
- 对动态对象更容易过期
- 决策链更难解释
- 小图上不一定有收益

当前方案更偏向：

- 共享最稳定、最值钱的信息
- 尽量不引入大量易过期信息

## 13. Benchmark 方法

仓库中已经补了 benchmark 工具：

- `benchmark/seed-groups.json`
- `benchmark/run-seed-groups.ps1`
- `benchmark/results/`

这套工具的作用是：

- 固定几组 seed
- 对不同版本做可重复的对比
- 避免只靠随机 10-run 平均去判断策略好坏

这在当前项目里很重要，因为：

- 大图分数有波动
- 有些失败 seed 会造成灾难性掉分
- 没有固定 seed 对照，很难判断某次改动是真提升还是随机波动

## 14. 相对初始版本的核心改动

如果从 starter code 往后看，当前完整方案的关键变化可以概括成：

1. 从单个弱 agent，扩展成默认 `6` agent 的同构团队
2. 从几乎无高层记忆，改成显式维护 tile/hole/fuel 的战略记忆
3. 从最近目标贪心，改成分层筛选和精排
4. 从弱 fuel 控制，改成 fuel 预算驱动的目标可达性判断
5. 从单一扫描，改成小图 macro sweep、大图 pre-fuel 搜站加 sector exploration
6. 从无协作，改成 fuel station 共享 + sector 共享 + sector claim

## 15. 当前方案的局限

虽然当前版本已经比起初始状态强很多，但仍有明确限制。

### 15.1 大图 pre-fuel 的残余风险

当前这个 `pre-fuel sector search` 分支，本来就是为了降低 `80 x 80` 下那种“第一次迟迟找不到 fuel station，最后全队跑空”的灾难性 seed。

按开发时使用的三组固定 benchmark seed 来看，这一版已经没有再复现之前那几个典型的全队 fuel-out case。

但这不等于理论上完全消失：

- fuel station 仍然只有一个，而且开局未知
- agent 的出生点仍然是随机的
- 早期搜索质量仍然会受 seed 影响

所以更准确的说法应该是：

- 在当前固定 benchmark 组里，这类 pre-fuel 灾难性失败已经没有再出现
- 但在未测试过的新 seed 上，它仍然是一个理论上的边缘风险

### 15.2 主区分工是静态的

主区按 agent index 固定，而不是按出生点动态调整。

优点是简单稳定，缺点是：

- agent 可能离自己的区很远
- 这会在某些 seed 下拖慢早期有效探索

### 15.3 共享 sector 统计还没有完全吃满

虽然共享 snapshot 会把 tile/hole 计数带过来，但当前真正参与决策的仍以本地统计和保守 freshness 融合为主。

这是一种保守实现，目的是先保证稳定。

### 15.4 还没有做 tile/hole 级任务认领

当前只有 sector 级 claim，没有 tile 级或 hole 级 ownership。

所以在边界区域附近，仍可能出现少量重复追逐。

## 16. 设计取舍

当前方案明确偏向：

- 可解释
- 每步计算代价可控
- 稳定覆盖
- 轻量协作

而没有走向：

- 集中式最优规划
- 全局拍卖
- 重通信
- 多 agent 全局路径规划

这种取舍在本项目里是合理的，因为：

- 小图上稳定性更重要
- 大图上 pre-fuel 搜站和 sector 层已经能带来明显收益
- 策略整体仍然容易说明和复现实验

## 17. 总结

当前 `wip/prefuel-search-save` 分支的方法可以概括为：

- 6 个同构 agent 共享一套反应式策略
- 每个 agent 有主区，用于粗粒度空间分工
- 所有 agent 都维护 tile、hole、fuel、sector 的战略记忆
- fuel station 发现后全队共享
- 大图下先用 pre-fuel sector 搜站完成第一次 fuel 发现，再用 sector freshness 和机会值调度探索
- 通过 sector snapshot 和 sector claim 实现轻量通信
- 小图保持稳定的 macro sweep
- 大图在 fuel station 已知后切换到主区约束下的 sector exploration

这就是当前项目根目录下 `wip/prefuel-search-save` 分支所实现的方法。
