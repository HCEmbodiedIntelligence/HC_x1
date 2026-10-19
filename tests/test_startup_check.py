import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest

from hc_x1.startup_check import Readiness, expected_joints, inspect_log, wait_for_startup

ROOT = Path(__file__).resolve().parents[1]
FAULT = '[ERROR] 控制模式验证失败：A(ERR=6, State=100, ImpType=1)，B(ERR=0, State=3, ImpType=1)\n'


class StartupCheckTests(unittest.TestCase):
    def test_actual_fault_is_reported_before_any_ready_decision(self):
        with tempfile.TemporaryDirectory() as tmp:
            log = Path(tmp) / 'upper_body.log'
            log.write_text(FAULT + '[ERROR] Marvin 机器人连接失败，节点将继续运行\n')
            lines = []
            result = wait_for_startup(log, [os.getpid()], Readiness(set()),
                                      lambda _: self.fail('failure should be immediate'), 3, False,
                                      output=lines.append)
            self.assertEqual(result, 1)
            self.assertIn('A(ERR=6, State=100', '\n'.join(lines))
            self.assertIn(str(log), '\n'.join(lines))
            # The CLI also reports this without importing rclpy or accessing SDK.
            result = subprocess.run([sys.executable, str(ROOT / 'hc_x1/startup_check.py'),
                                     '--log', str(log)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 1)
            self.assertIn('ERR=6', result.stdout)

    def test_mode_specific_joint_names(self):
        config = {'left_arm': {'arm_joints': [f'L{i}' for i in range(7)]},
                  'right_arm': {'arm_joints': [f'R{i}' for i in range(7)]},
                  'waist': {'joints': ['w1', 'w2', 'w3']}}
        self.assertEqual(len(expected_joints(config, [])), 17)
        self.assertEqual(expected_joints(config, ['arms:=false']), {'w1', 'w2', 'w3'})
        self.assertEqual(len(expected_joints(config, ['waist:=false'])), 14)
        self.assertEqual(len(expected_joints(config, ['arms:=false', 'arms:=true'])), 17)
        config['waist']['enabled'] = False
        self.assertEqual(len(expected_joints(config, [])), 14)

    def test_feedback_requires_complete_finite_unique_and_fresh_samples(self):
        state = Readiness({'L', 'R'})
        state.ready_message(SimpleNamespace(data=True), 10.)
        for names, positions in [(['L'], [0.]), (['L', 'L'], [0., 0.]),
                                 (['L', 'R'], [0., math.nan]), (['L', 'R'], [0.])]:
            state.joint_message(SimpleNamespace(name=names, position=positions), 10.)
            self.assertFalse(state.available(10.1))
        state.joint_message(SimpleNamespace(name=['L', 'R'], position=[.1, .2]), 10.)
        self.assertTrue(state.available(10.1))
        self.assertFalse(state.available(11.1))
        state.ready_message(SimpleNamespace(data=False), 10.2)
        self.assertFalse(state.available(10.2))

    def test_waist_home_message_is_not_whole_robot_home_completion(self):
        self.assertFalse(inspect_log('[node]: 腰部回零完成')[1])
        self.assertTrue(inspect_log('[node]: 回零完成')[1])
        self.assertTrue(inspect_log('[node]: 回零完成（双臂未连接，仅腰部）')[1])

    def test_timeout_never_announces_success(self):
        with tempfile.TemporaryDirectory() as tmp:
            clock = [0.]
            lines = []
            def spin(duration):
                clock[0] += duration
            result = wait_for_startup(Path(tmp) / 'missing.log', [os.getpid()], Readiness({'L'}),
                                      spin, .5, True, clock=lambda: clock[0], output=lines.append)
            self.assertEqual(result, 1)
            self.assertIn('joint_states', '\n'.join(lines))
            self.assertIn('回零完成', '\n'.join(lines))
            self.assertNotIn('检查通过', '\n'.join(lines))

    def test_transient_ready_before_homing_is_not_success(self):
        with tempfile.TemporaryDirectory() as tmp:
            log = Path(tmp) / 'upper_body.log'
            log.write_text('启动后自动回零至 rest_j_pos\n')
            clock, lines = [0.], []
            state = Readiness({'L'})
            def spin(duration):
                clock[0] += duration
                state.ready_message(SimpleNamespace(data=clock[0] < .3 or clock[0] > 1.), clock[0])
                state.joint_message(SimpleNamespace(name=['L'], position=[0.]), clock[0])
                if clock[0] > 1.:
                    log.write_text('启动后自动回零至 rest_j_pos\n[node]: 回零完成\n')
            result = wait_for_startup(log, [os.getpid()], state, spin, 5, True,
                                      clock=lambda: clock[0], output=lines.append)
            self.assertEqual(result, 0)
            self.assertGreaterEqual(clock[0], 3.)

    def test_launcher_exits_and_cleans_fake_drivers_on_initialization_error(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            for name in ('install', 'hc_x1', 'src/hc_tj', 'bin'):
                (root / name).mkdir(parents=True)
            script = (ROOT / 'start.sh').read_text().replace('/tmp/hc_x1_hardware.lock', str(root / 'lock'))
            script = script.replace('source /opt/ros/humble/setup.bash', ': # test ROS environment')
            (root / 'start.sh').write_text(script)
            (root / 'install/setup.bash').write_text('')
            (root / 'bin/pgrep').write_text('#!/bin/sh\nexit 1\n')
            (root / 'bin/pgrep').chmod(0o755)
            driver = root / 'src/hc_tj/run_hc_tj_all.sh'
            driver.write_text('#!/bin/bash\nmkdir -p "$HC_TJ_LOG_DIR"\n'
                              'echo "$$" > "$HC_TJ_LOG_DIR/driver.pid"\n'
                              "echo '" + FAULT.strip() + "' > \"$HC_TJ_LOG_DIR/upper_body.log\"\n"
                              'sleep 60 &\nwait\n')
            for name in ('control_adapter.py', 'joint_command_smoother.py'):
                (root / 'hc_x1' / name).write_text('import time\ntime.sleep(60)\n')
            # Run the real read-only checker loop without ROS subscriptions, while
            # the shell owns only stub hardware processes in this temporary tree.
            (root / 'hc_x1/startup_check.py').write_text(
                'import sys,time,argparse\n'
                f'sys.path.insert(0,{str(ROOT)!r})\n'
                'from hc_x1.startup_check import Readiness,wait_for_startup\n'
                'from pathlib import Path\n'
                'p=argparse.ArgumentParser();p.add_argument("--log",type=Path);p.add_argument("--pid",type=int,action="append");a,_=p.parse_known_args()\n'
                'sys.exit(wait_for_startup(a.log,a.pid,Readiness(set()),time.sleep,5,False))\n')
            env = dict(os.environ, PATH=str(root / 'bin') + ':' + os.environ['PATH'],
                       HC_X1_LOG_DIR=str(root / 'runtime'), HC_X1_STARTUP_TIMEOUT_SEC='5')
            process = subprocess.Popen(['bash', str(root / 'start.sh'), '--no-camera', '--no-hands', '--no-chassis'],
                                       env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                       text=True, start_new_session=True)
            try:
                output, _ = process.communicate(timeout=10)
            finally:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            self.assertNotEqual(process.returncode, 0, output)
            self.assertIn('ERR=6', output)
            self.assertIn('HC_X1 启动失败', output)
            self.assertNotIn('HC_X1 已启动；', output)
            child = int((root / 'runtime/drivers/driver.pid').read_text())
            self.assertFalse(Path(f'/proc/{child}').exists(), output)


if __name__ == '__main__':
    unittest.main()
