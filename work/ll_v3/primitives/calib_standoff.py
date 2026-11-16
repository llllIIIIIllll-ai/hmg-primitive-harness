def run(api, standoffs=(0.35, 0.30, 0.25, 0.20, 0.15),
        gap=0.155, z_off=0.45, x_off=0.0, height=None,
        max_bursts=50, burst=0.5, speed=0.20, **kw):
    if height is not None:
        api.set_height(height, 1.2)
    out = []
    for target_so in standoffs:
        k = 0
        while k < max_bursts:
            st = api.observe()
            ob = st.get('object') or {}
            b = api.base_state()
            a = api.object_aabb()
            lo = np.array(a['lo'])
            hi = np.array(a['hi'])
            bp = np.array(b['pos'])
            near = np.clip(bp, lo, hi)
            dist_aabb = np.linalg.norm(bp - near)
            if dist_aabb <= target_so + 0.02:
                break
            dx = ob['pos'][0] - b['pos'][0]
            dy = ob['pos'][1] - b['pos'][1]
            yaw = np.radians(b['yaw_deg'])
            dx_r = dx * np.cos(yaw) + dy * np.sin(yaw)
            dy_r = -dx * np.sin(yaw) + dy * np.cos(yaw)
            yaw_err = np.arctan2(dy_r, dx_r)
            while yaw_err > 3.14159:
                yaw_err = yaw_err - 6.28318
            while yaw_err < -3.14159:
                yaw_err = yaw_err + 6.28318
            wz = np.clip(0.6 * yaw_err, -0.15, 0.15)
            vy = np.clip(0.5 * dy_r, -0.08, 0.08)
            vx = np.clip(0.5 * (dist_aabb - target_so), 0.05, speed)
            api.move_base(x=vx, y=vy, yaw=wz, dur=burst)
            k = k + 1
        st = api.observe()
        b = api.base_state()
        ob = st.get('object') or {}
        a = api.object_aabb()
        lo = np.array(a['lo'])
        hi = np.array(a['hi'])
        bp = np.array(b['pos'])
        near = np.clip(bp, lo, hi)
        dist_aabb = np.linalg.norm(bp - near)
        dx = ob['pos'][0] - b['pos'][0]
        dy = ob['pos'][1] - b['pos'][1]
        dist_root = (dx * dx + dy * dy) ** 0.5
        r = api.reach_box_qp(task_dim=12, gap=gap, z_off=z_off,
                             x_off=x_off, dur=1.0, iters=80)
        pad_err = {}
        for side in (r.get('palm_residual') or {}):
            v = r['palm_residual'][side]
            pad_err[side] = (v[0] * v[0] + v[1] * v[1]
                             + v[2] * v[2]) ** 0.5
        out.append({'target_standoff': target_so,
                    'achieved_standoff_aabb': dist_aabb,
                    'achieved_standoff_root': dist_root,
                    'bursts': k, 'base': b,
                    'object': ob.get('pos'),
                    'object_aabb': {'lo': a['lo'], 'hi': a['hi']},
                    'reach': r, 'pad_error': pad_err,
                    'pad_error_definition':
                        'executed palm/site residual (harness '
                        'palm_residual); the pad is rigidly offset'})
    return {'measurements': out, 'height': height}
