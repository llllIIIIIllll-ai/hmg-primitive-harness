def run(api, xs=(-0.04, 0.02), gaps=(0.155, 0.175),
        zs=(0.46, 0.52, 0.58, 0.64, 0.70), orients=('down',),
        iters=150, **kw):
    return api.ik_preview(xs=xs, gaps=gaps, zs=zs, orients=orients,
                          iters=iters, sides=('left', 'right'))
