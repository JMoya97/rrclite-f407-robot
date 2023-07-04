#include "object.h"

TypeObjectTypeDef type_type_instant = {.type = &type_type_instant, .type_id = OBJECT_TYPE_ID_TYPE };

TypeObjectTypeDef type_key_event_instant = { .type = &type_type_instant, .type_id = OBJECT_TYPE_ID_KEY_EVENT };

TypeObjectTypeDef type_sbus_status_instant = { .type = &type_type_instant, .type_id = OBJECT_TYPE_ID_SBUS_STATUS };

TypeObjectTypeDef type_gamepad_status_instant = {.type = &type_type_instant, .type_id = OBJECT_TYPE_ID_GAMEPAD_STATUS };

