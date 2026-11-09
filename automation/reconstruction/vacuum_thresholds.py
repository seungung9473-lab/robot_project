"""Single place for the suction pressure threshold used by the pick/place scripts (raw ESP32 diff, not kPa).

HELD_DIFF: diff at or below this = the cup is sealed / holding. Above it = not held (empty cup).
2026-11-09 사용자 확인: diff -100까지도 물체가 잘 붙어 있어 기존 -200에서 -100으로 낮춤.
Release confirmation (diff > -50) is separate and unchanged; the bridge reads the same value from soarm.yaml.
"""
HELD_DIFF=-100
RELEASE_CONFIRM_DIFF=-50
