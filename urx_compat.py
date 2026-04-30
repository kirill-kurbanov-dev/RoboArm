def _flatten(values):
    result = []
    for value in values:
        if isinstance(value, (list, tuple)):
            result.extend(_flatten(value))
        else:
            result.append(value)
    return result


def _to_plain_list(value):
    if hasattr(value, "tolist"):
        return value.tolist()
    try:
        return list(value)
    except TypeError:
        return [value]


def patch_urx_math3d():
    try:
        import math3d as m3d
    except Exception:
        return

    candidates = []
    for attr in ("PoseVector", "Vector"):
        cls = getattr(m3d, attr, None)
        if cls is not None:
            candidates.append(cls)

    try:
        transform = m3d.Transform()
        candidates.append(type(transform.pose_vector))
    except Exception:
        pass

    def tolist(self):
        for attr in ("array", "data"):
            if not hasattr(self, attr):
                continue
            value = getattr(self, attr)
            if callable(value):
                value = value()
            plain = _to_plain_list(value)
            return [float(item) for item in _flatten(plain)]

        plain = _to_plain_list(self)
        return [float(item) for item in _flatten(plain)]

    for cls in set(candidates):
        if not hasattr(cls, "tolist"):
            setattr(cls, "tolist", tolist)
