# grasp_verify -- READ-ONLY post-close grasp check for 05_drill_lift.
# Failure signature it addresses (measured in v34a): seed 7 closes its fingers
# on air (0 gripper0 contact rows, thumb_2-to-index_1 rel-z gap 0.0108) while
# every lifting seed at the same instant has 3-5 gripper0 contact rows and a
# gap of 0.068-0.083.  No pre-hoc quantity separates them (br.dev and every
# pre_stance geometry separate by only 4-44 mm on n=1), so the routing decision
# must be made from THIS measured post-close state instead of a prediction.
# It issues no motion: hand_geoms()/observe() only run mj_forward on live data
# and contact_detail() only reads the contact list, so qpos/qvel and the
# episode's command stream are untouched.
def run(api, match='bottle', n_min=2, gap_min=0.040, **kw):
    ob = ((api.observe() or {}).get('object') or {}).get('pos')
    out = {'n_min': n_min, 'gap_min': gap_min, 'obj': ob,
           't': np.round(api.t, 2).item()}
    rel = {}
    if ob is not None:
        hg = api.hand_geoms() or {}
        for b in hg:
            if '_left_' not in b:
                continue
            for gg in hg[b]['geoms']:
                nm = gg['name']
                if nm[-4:] != '_col':
                    continue
                w = gg['world_pos']
                rel[nm] = [np.round(w[0] - ob[0], 4).item(),
                           np.round(w[1] - ob[1], 4).item(),
                           np.round(w[2] - ob[2], 4).item()]
    th = None
    ix = None
    for nm in rel.keys():
        if nm.find('thumb_2_link_col') >= 0:
            th = rel[nm]
        if nm.find('index_1_link_col') >= 0:
            ix = rel[nm]
    gap = None
    if th is not None and ix is not None:
        gap = np.round(th[2] - ix[2], 4).item()
    out['gap'] = gap
    out['thumb2'] = th
    out['index1'] = ix
    cd = api.contact_detail(limit=40, match=match) or []
    rows = []
    n_fing = 0
    fmax = 0.0
    nb = 0
    for c in cd:
        nb += 1
        tag = (c.get('b1') or '') + '|' + (c.get('b2') or '')
        gtag = (c.get('g1') or '') + '|' + (c.get('g2') or '')
        isf = 0
        if gtag.find('gripper0') >= 0:
            isf = 1
            n_fing += 1
        fr = c.get('force')
        fv = None if fr is None else np.round(fr[0], 4).item()
        if isf and fv is not None and fv > fmax:
            fmax = fv
        rows.append({'b': tag, 'g': gtag, 'f': fv, 'isf': isf,
                     'p': c.get('pos')})
    out['n_fing'] = n_fing
    out['fmax'] = np.round(fmax, 4).item()
    out['n_bc'] = nb
    out['c'] = rows
    held = 0
    if n_fing >= n_min:
        held = 1
    if gap is not None and gap < gap_min:
        held = 0
    out['held'] = held
    return out
