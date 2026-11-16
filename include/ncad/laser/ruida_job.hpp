// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

// A planned LaserJob into the Ruida encoder's input. Kept apart from both so
// that job.hpp stays controller-independent and the encoder knows nothing of
// pens, ACI or .ljob.
#pragma once

#include "ncad/laser/job.hpp"
#include "ncad/laser/ruida_encode.hpp"

#include <string>

namespace ncad::laser {

// Refuses an unplanned job: drivers accept only planned ones, so joining and
// ordering cannot be skipped by accident.
bool job_to_rd(const LaserJob& planned, ruida::RdJob& out, std::string& error);

// The job's box in its own coordinates; false for a job with no points.
bool job_bounds(const LaserJob& job, Pt& min, Pt& max);

}  // namespace ncad::laser
