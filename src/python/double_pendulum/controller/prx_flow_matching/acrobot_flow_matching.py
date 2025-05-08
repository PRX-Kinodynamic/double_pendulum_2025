import timeit
import numpy as np
import torch
import math
from double_pendulum.controller.abstract_controller import AbstractController
from genMoPlan.models import GenerativeModel
from genMoPlan.utils import load_model, get_normalizer_params
from genMoPlan.datasets.normalization import LimitsNormalizer
import time

class AcrobotFlowMatchingController(AbstractController):
    def __init__(self, model_path, horizon_length=None, integration_steps=5, integration_method='euler'):
        super().__init__()
        self.use_gravity_compensation = False
        self.model, self.model_args = load_model(model_path)
        self.normalizer = LimitsNormalizer(params=get_normalizer_params(self.model_args))
        self.history_length: int = self.model_args.history_length
        self.horizon_length: int = horizon_length if horizon_length is not None else self.model_args.horizon_length
        self.stride: int = self.model_args.stride
        self.action_index: int = self.model_args.action_indices[0]
        self.integration_steps: int = integration_steps
        self.integration_method: str = integration_method
        self.history_buffer = torch.zeros((self.history_length, 5), dtype=torch.float32)
        
        self.step = 0
        self.prev_t = 0
        self.horizon_left = 0
        self.torque_limit = 6
        
        self.u = 0
        self.first_call = True
        self.normalized_predicted_horizon = None
        
        # Profiling variables
        self.all_times = []
        self.inference_times = []
        self.non_inference_times = []

    def update_history_buffer(self, x):
        # Shift history buffer up, removing first entry
        self.history_buffer = self.history_buffer.roll(-1, dims=0)

        new_state = np.concatenate([x, [self.u]])
        normalized_state = self.normalizer(new_state[None, :])
        self.history_buffer[-1, :] = torch.tensor(normalized_state[0], dtype=torch.float32)

    def get_conditions(self):
        cond = {}
        for i in range(self.history_length):
            cond[i] = self.history_buffer[i].unsqueeze(0)

        return cond

    def compute_new_control(self):
        cond = self.get_conditions()
        sample = self.model(cond, integration_steps=self.integration_steps, integration_method=self.integration_method)
        self.normalized_predicted_horizon = sample.trajectories.to('cpu').numpy()[:, self.history_length:, :]
        self.horizon_left = self.horizon_length

    def update_control(self, x):
        self.update_history_buffer(x)

        if self.first_call:
            self.compute_new_control()
            self.first_call = False
        
        # Get control from model
        model_control = 0.0
        if self.normalized_predicted_horizon is not None:
            state_idx = self.horizon_length - self.horizon_left
            state = self.normalized_predicted_horizon[:, state_idx:state_idx+1, :]
            model_control = self.normalizer.unnormalize(state)[0,0, self.action_index]
        
        control = model_control
        
        self.u = np.clip(control, -self.torque_limit, self.torque_limit)
        
        self.horizon_left -= 1

        if self.horizon_left == 0:
            self.compute_new_control()
        
    def report_profiling_stats(self):
        """
        Generates and prints profiling statistics for the controller.
        Call this method after the experiment is complete.
        """
        if not self.all_times:
            print("No profiling data collected.")
            return
            
        avg_time = sum(self.all_times) / len(self.all_times)
        avg_freq = 1.0 / avg_time if avg_time > 0 else 0
        
        inference_avg = sum(self.inference_times) / len(self.inference_times) if self.inference_times else 0
        inference_freq = 1.0 / inference_avg if inference_avg > 0 else 0
        
        non_inference_avg = sum(self.non_inference_times) / len(self.non_inference_times) if self.non_inference_times else 0
        non_inference_freq = 1.0 / non_inference_avg if non_inference_avg > 0 else 0
        
        print("--- Controller Profiling Statistics ---")
        print(f"Total calls: {len(self.all_times)}")
        print(f"Overall: Avg time: {avg_time*1000:.2f} ms, Freq: {avg_freq:.2f} Hz")
        print(f"With inference ({len(self.inference_times)} calls): Avg time: {inference_avg*1000:.2f} ms, Freq: {inference_freq:.2f} Hz")
        print(f"Without inference ({len(self.non_inference_times)} calls): Avg time: {non_inference_avg*1000:.2f} ms, Freq: {non_inference_freq:.2f} Hz")
        
        # Calculate min/max for overall execution time
        if self.all_times:
            max_time = max(self.all_times) * 1000
            print(f"Min execution time: 0.00 ms, Max execution time: {max_time:.2f} ms")
        
        # Statistics for inference and non-inference calls
        if self.inference_times:
            min_inf = min(self.inference_times) * 1000
            max_inf = max(self.inference_times) * 1000
            print(f"Inference calls: Min: {min_inf:.2f} ms, Max: {max_inf:.2f} ms")
            
        if self.non_inference_times:
            min_non_inf = min(self.non_inference_times) * 1000
            max_non_inf = max(self.non_inference_times) * 1000
            print(f"Non-inference calls: Min: {min_non_inf:.2f} ms, Max: {max_non_inf:.2f} ms")
            
        print("---------------------------------------")

    def get_control_output_(self, x, t=None):
        """
        The function to compute the control input for the double pendulum's
        actuator(s).

        Parameters
        ----------
        x : array_like, shape=(4,), dtype=float,
            state of the double pendulum,
            order=[angle1, angle2, velocity1, velocity2],
            units=[rad, rad, rad/s, rad/s]
        t : float, optional
            time, unit=[s]
            (Default value=None)

        Returns
        -------
        array_like
            shape=(2,), dtype=float
            actuation input/motor torque,
            order=[u1, u2],
            units=[Nm]
        """
        self.use_gravity_compensation = True
        start_time = timeit.default_timer()
        
        # Track if this call includes model inference
        has_inference = self.horizon_left == 0 and self.step % self.stride == 0
        
        if self.step % self.stride == 0:
            self.update_control(x)
        u = [0.0, self.u]

        self.step += 1

        end_time = timeit.default_timer()
        execution_time = end_time - start_time
        
        # Log profiling data
        self.all_times.append(execution_time)
        if has_inference:
            self.inference_times.append(execution_time)
        else:
            self.non_inference_times.append(execution_time)
        
        return u