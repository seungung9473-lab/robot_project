"""Single place for the suction pressure threshold used by the pick/place scripts (raw ESP32 diff, not kPa).

HELD_DIFF: diff at or below this = the cup is sealed / holding. Above it = not held (empty cup).
2026-11-09 사용자 확인: diff -100까지 붙어 있어 -200에서 -100으로 낮췄으나, -106에서 겨우 붙은 약한 밀착이 들어 올리다 풀려(23회차) -150으로 올림.
Release confirmation (diff > -50) is separate and unchanged; the bridge reads the same value from soarm.yaml.
"""
HELD_DIFF=-150
RELEASE_CONFIRM_DIFF=-50
