# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, Sadie Forbes
#
# Regenerates tests/fixtures/meerk40t_square.rd by driving MeerK40t's own RDJob
# writer methods. Development only; never shipped or run by the tests.
#
#   python3 -I tools/mk_meerk40t_rd.py ../meerk40t tests/fixtures/meerk40t_square.rd
#
# numpy/PIL are stubbed: rdjob.py imports them transitively but these writer
# methods never touch them.
import sys
sys.path.insert(0, sys.argv[1])
import types
class _Stub(types.ModuleType):
    def __getattr__(self, n): return _Stub(n)
    def __call__(self, *a, **k): return _Stub("x")
for _m in ("numpy", "PIL", "PIL.Image", "PIL.ImageOps", "PIL.ImageFilter"):
    sys.modules.setdefault(_m, _Stub(_m))
from meerk40t.ruida.rdjob import RDJob
j = RDJob(magic=0x88)
# A 50 mm square at (10,10) mm, two layers, written through MeerK40t's own methods.
j.ref_point_2(); j.set_absolute(); j.ref_point_set(); j.enable_block_cutting(0)
j.start_process(); j.feed_repeat(0, 0); j.set_feed_auto_pause(0)
j.process_top_left(60000, 10000); j.process_bottom_right(10000, 60000)
j.document_min_point(60000, 10000); j.document_max_point(10000, 60000)
j.process_repeat(1, 1, 0, 0, 0, 0, 0); j.array_direction(0)
for part, (speed, power, color) in enumerate([(150, 18, 0xFF0000), (20, 65, 0x0000FF)]):
    j.speed_laser_1_part(part, speed)
    j.min_power_1_part(part, power); j.max_power_1_part(part, power)
    j.min_power_2_part(part, power); j.max_power_2_part(part, power)
    j.layer_color_part(part, color); j.work_mode_part(part, 0)
    j.part_min_point(part, 10000, 10000); j.part_max_point(part, 60000, 60000)
    j.part_min_point_ex(part, 10000, 10000); j.part_max_point_ex(part, 60000, 60000)
j.max_layer_part(1)
j.pen_offset(0, 0); j.pen_offset(1, 0); j.layer_offset(0, 0); j.layer_offset(1, 0); j.display_offset(0, 0)
j.feed_info(0)
j.array_start(0); j.set_current_element_index(0); j.array_en_mirror_cut(0)
j.array_min_point(10000, 10000); j.array_max_point(60000, 60000); j.array_add(0, 0); j.array_mirror(0)
j.array_repeat(1, 1, 0, 1123, -3328, 4, 3480)
j.first_move = True
for part, (speed, power) in enumerate([(150, 18), (20, 65)]):
    j.write_settings({"part": part, "speed": speed, "power": power * 10, "coolant": 1})
    j.x, j.y = 0, 0
    pts = [(10000, 10000), (60000, 10000), (60000, 60000), (10000, 60000), (10000, 10000)]
    if part == 1:
        pts = [(20000, 20000), (28192, 20000), (28192, 30000)]  # includes a +8192 relative step
    x, y = pts[0]
    j.jump(x, y, x - j.x, y - j.y); j.x, j.y = x, y
    for (nx, ny) in pts[1:]:
        j.mark(nx, ny, nx - j.x, ny - j.y); j.x, j.y = nx, ny
j.write_tail()
open(sys.argv[2], "wb").write(j.swizzle(j.get_contents()))
