/*
 * PMSF FMU Framework for FMI 2.0 Co-Simulation FMUs
 *
 * (C) 2016 -- 2018 PMSF IT Consulting Pierre R. Mai
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#cmakedefine PUBLIC_LOGGING
#cmakedefine PRIVATE_LOG_PATH "@PRIVATE_LOG_PATH@"
#cmakedefine VERBOSE_FMI_LOGGING
#cmakedefine DEBUG_BREAKS
#define FMU_GUID "@FMUGUID@"

#cmakedefine FMU_LISTEN
#define FMU_DEFAULT_ADDRESS "@FMU_DEFAULT_ADDRESS@"
#define FMU_DEFAULT_PORT "@FMU_DEFAULT_PORT@"
