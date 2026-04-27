#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机器狗 UDP 运动控制示例

这个示例面向机器狗巡检控制工程师，目标是帮助快速理解：

1. 机器狗如何通过 UDP 建立控制链路
2. 机器狗初始化流程如何调用
3. 运动控制本质上是 3 个轴指令的组合：
   - forward_speed: 前后移动
   - side_speed: 左右平移
   - turn_speed: 左右转向
4. 在“追逐选定目标”场景里，常用的是：
   - 前进
   - 左转/右转
   - 前进 + 转向 的组合
   - 停止

本示例默认不强调复杂算法，只把“目标追逐”的底层动作实现方式讲清楚。

用法：
    python udp_motion_demo.py
    python udp_motion_demo.py --forward-only
    python udp_motion_demo.py --forward-only --quick
    python udp_motion_demo.py --ip 10.69.235.139 --port 43893
"""

import argparse
import sys
import time

from totalController import Controller


class SoftExitController(Controller):
    """
    与 Controller 能力相同；用于 `with` 结束时只做零轴 + 停心跳 + 关 socket，
    不调用 emergency_stop（急停容易让狗趴下，不利调试「做完一个动作仍保持站立」）。
    """

    def close(self):
        print("\n关闭控制器（软关闭，无急停）...")
        self.stop_continuous_move()
        self.stop()
        # 必须在 stop_heartbeat 之前关持续运动：部分固件无心跳时不处理 CONTINUOUS_MOTION，会导致下次 enable 无效
        try:
            self.disable_continuous_motion()
        except OSError:
            pass
        time.sleep(0.12)
        self.stop_heartbeat()
        self.stop()
        time.sleep(0.2)
        try:
            self.socket.close()
            print("✓ 控制器已关闭\n")
        except OSError as e:
            print(f"❌ 关闭socket失败: {e}\n")


DEFAULT_IP = "10.69.235.139"  # 按现场改；控制端需与狗网络互通
DEFAULT_PORT = 43893


def print_section(title):
    print("\n" + "=" * 72)
    print(title)
    print("=" * 72)


def prepare_robot_quick(robot):
    """狗已站立：仅心跳 + 移动/手动模式，不重复起立（见 track_json_one 说明）。"""
    try:
        robot.start_heartbeat(frequency=2.0)
        time.sleep(0.8)
        robot.switch_to_move_mode()
        time.sleep(0.5)
        robot.switch_to_manual_mode()
        time.sleep(0.4)
        return True
    except Exception as e:
        print(f"prepare_robot_quick 失败: {e}", file=sys.stderr)
        return False


def safe_stop(robot):
    """统一停止出口：停线程、零轴，并关闭持续运动通道（否则部分固件会保持速度刹不停）。"""
    robot.stop_continuous_move()
    robot.stop()
    try:
        robot.disable_continuous_motion()
    except OSError:
        pass
    time.sleep(0.05)
    robot.stop()


def run_axis_motion(robot, name, forward=0.0, side=0.0, turn=0.0, duration=2.0, pause=1.0):
    """
    演示一次底层轴控制。

    在 totalController.py 中：
    - move() / start_continuous_move() 最终会发送 MOVE_X / MOVE_Y / TURN 三类指令
    - 追逐目标通常主要使用 forward + turn
    """
    print(f"\n>>> {name}")
    print(
        f"发送控制量: forward={forward:.2f}, side={side:.2f}, turn={turn:.2f}, "
        f"duration={duration:.1f}s"
    )
    if not robot.move_running:
        robot.enable_continuous_motion()
        time.sleep(0.03)
    robot.start_continuous_move(
        forward_speed=forward,
        side_speed=side,
        turn_speed=turn,
    )
    time.sleep(duration)
    safe_stop(robot)
    time.sleep(pause)


def explain_motion_principle():
    print_section("运动控制原理")
    print("1. 不是一次发“前进动作”就一直生效，而是持续发送轴控制指令。")
    print("2. `forward_speed` 控制前后，正值前进，负值后退。")
    print("3. `turn_speed` 控制转向，正值右转，负值左转。")
    print("4. `side_speed` 控制横移，但追逐目标时通常不作为主控制量。")
    print("5. 追逐目标最常见的做法是：")
    print("   - 目标在正前方: 前进")
    print("   - 目标偏左: 左转或前进+左转")
    print("   - 目标偏右: 右转或前进+右转")
    print("   - 目标丢失/到达停止距离: 停止")


def initialize_robot(robot):
    print_section("步骤 1: 初始化机器狗")
    ok = robot.initialize()
    if not ok:
        raise RuntimeError("initialize() 返回失败，初始化未完成")


def demo_basic_actions(robot, move_speed, turn_speed, duration):
    print_section("步骤 2: 基础动作与底层控制对应关系")
    print("下面几步用于建立“动作名称”和“底层控制量”的对应关系。")

    run_axis_motion(
        robot,
        "前进: 只给 forward_speed 正值",
        forward=move_speed,
        duration=duration,
    )
    run_axis_motion(
        robot,
        "后退: 只给 forward_speed 负值",
        forward=-move_speed,
        duration=duration,
    )
    run_axis_motion(
        robot,
        "左转: 只给 turn_speed 负值",
        turn=-turn_speed,
        duration=duration,
    )
    run_axis_motion(
        robot,
        "右转: 只给 turn_speed 正值",
        turn=turn_speed,
        duration=duration,
    )


def demo_target_chasing(robot, move_speed, turn_speed, duration):
    print_section("步骤 3: 模拟“追逐选定目标”时常用动作")
    print("这里不引入视觉算法，只模拟视觉模块已经给出了目标相对位置。")
    print("目标追逐控制的核心不是新动作，而是基础轴控制的组合。")

    run_axis_motion(
        robot,
        "场景 A: 目标在正前方，直接前进",
        forward=move_speed,
        duration=duration,
    )
    run_axis_motion(
        robot,
        "场景 B: 目标偏左，采用“前进 + 左转”",
        forward=move_speed,
        turn=-turn_speed,
        duration=duration,
    )
    run_axis_motion(
        robot,
        "场景 C: 目标偏右，采用“前进 + 右转”",
        forward=move_speed,
        turn=turn_speed,
        duration=duration,
    )

    print("\n>>> 场景 D: 到达停止距离或目标丢失，停止")
    safe_stop(robot)
    time.sleep(1.0)


def print_summary():
    print_section("结论")
    print("追逐目标时，工程上最关键的是把感知结果转换为 forward_speed 和 turn_speed。")
    print("也就是说：")
    print("1. 判断目标在左还是右 -> 决定 turn_speed 正负")
    print("2. 判断目标远近 -> 决定 forward_speed 大小")
    print("3. 目标居中且较远 -> 以前进为主")
    print("4. 目标偏移明显 -> 以前进+转向组合修正")
    print("5. 到达目标或需要急停 -> 发送停止指令")


def main():
    parser = argparse.ArgumentParser(description="机器狗 UDP 运动控制教学示例")
    parser.add_argument("--ip", default=DEFAULT_IP, help="机器狗 IP")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="机器狗 UDP 端口")
    parser.add_argument("--duration", type=float, default=2.0, help="每个演示动作持续时间，单位秒")
    parser.add_argument("--speed", type=float, default=0.3, help="前后移动速度，建议 0.2~0.4")
    parser.add_argument("--turn-speed", type=float, default=0.3, help="左右转向速度，建议 0.2~0.4")
    parser.add_argument(
        "--forward-only",
        action="store_true",
        help="仅初始化后直线前进一段再停止（真机烟测用）",
    )
    parser.add_argument(
        "--quick",
        action="store_true",
        help="配合 --forward-only：狗已站立时跳过完整 initialize，不重复起立",
    )
    args = parser.parse_args()

    print_section("连接信息")
    print(f"目标机器狗: {args.ip}:{args.port}")

    with SoftExitController((args.ip, args.port)) as robot:
        try:
            if args.forward_only:
                print("模式: 仅前进" + ("（--quick）" if args.quick else ""))
                if args.quick:
                    if not prepare_robot_quick(robot):
                        raise RuntimeError("prepare_robot_quick 失败")
                else:
                    initialize_robot(robot)
                run_axis_motion(
                    robot,
                    "前进",
                    forward=args.speed,
                    duration=args.duration,
                    pause=0.5,
                )
                print("前进结束。")
                return

            print("本程序将依次演示：初始化 -> 基础动作 -> 目标追逐动作组合 -> 停止")
            explain_motion_principle()
            initialize_robot(robot)
            demo_basic_actions(robot, args.speed, args.turn_speed, args.duration)
            demo_target_chasing(robot, args.speed, args.turn_speed, args.duration)
            print_summary()
        except KeyboardInterrupt:
            print("\n收到中断，正在停止机器狗...")
            safe_stop(robot)
            raise
        except Exception:
            safe_stop(robot)
            raise


if __name__ == "__main__":
    main()
