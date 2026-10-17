#pragma once

/// @brief State of charge (0-100 %) of a TRMNL X cell estimated from its resting voltage.
int estimate_soc_from_voltage(float voltage);
