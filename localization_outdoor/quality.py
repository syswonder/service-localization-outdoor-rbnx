import math


def fix_grade(fix, config):
    if not fix or not fix.get("valid") or not fix.get("differential"):
        return None
    hacc = fix.get("hacc_m")
    if (
        isinstance(hacc, bool)
        or not isinstance(hacc, (int, float))
        or not math.isfinite(hacc)
        or hacc <= 0
    ):
        return None
    if not (
        -90 <= fix.get("latitude", math.inf) <= 90
        and -180 <= fix.get("longitude", math.inf) <= 180
    ):
        return None
    age = fix.get("correction_age_s")
    if config["require_correction_age"] and (
        not isinstance(age, (int, float))
        or isinstance(age, bool)
        or not math.isfinite(age)
        or not 0 <= age <= config["correction_max_age_s"]
    ):
        return None
    if (
        fix.get("fixed")
        and fix.get("gga_quality") == 4
        and hacc <= config["gnss_fixed_hacc_m"]
    ):
        return "fixed"
    if (
        fix.get("float")
        and fix.get("gga_quality") == 5
        and hacc <= config["gnss_float_hacc_m"]
    ):
        return "float"
    return None
