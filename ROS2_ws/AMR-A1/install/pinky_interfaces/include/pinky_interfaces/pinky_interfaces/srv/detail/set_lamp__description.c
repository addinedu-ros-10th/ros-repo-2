// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from pinky_interfaces:srv/SetLamp.idl
// generated code does not contain a copyright notice

#include "pinky_interfaces/srv/detail/set_lamp__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_pinky_interfaces
const rosidl_type_hash_t *
pinky_interfaces__srv__SetLamp__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x23, 0x31, 0x1d, 0x28, 0xbd, 0x46, 0x8a, 0x70,
      0xd4, 0x7a, 0x2c, 0x4f, 0xd4, 0xaa, 0x5c, 0x05,
      0xb3, 0xa4, 0xaa, 0x98, 0x7e, 0xcf, 0x77, 0x67,
      0x07, 0x86, 0xd8, 0x0f, 0xce, 0x62, 0x80, 0xca,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_pinky_interfaces
const rosidl_type_hash_t *
pinky_interfaces__srv__SetLamp_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x84, 0x6e, 0xd7, 0x2b, 0x64, 0x72, 0x78, 0xa7,
      0xae, 0x6d, 0x50, 0x18, 0x97, 0x51, 0x15, 0xf3,
      0x69, 0x8e, 0x10, 0x71, 0x27, 0x43, 0xbb, 0xec,
      0x87, 0x81, 0x1d, 0x00, 0x46, 0x14, 0xc3, 0x0b,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_pinky_interfaces
const rosidl_type_hash_t *
pinky_interfaces__srv__SetLamp_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x01, 0x89, 0x81, 0x4b, 0x5b, 0xd6, 0xb2, 0x76,
      0xda, 0x94, 0x33, 0x7a, 0x89, 0x1c, 0x2b, 0x09,
      0x3e, 0x4b, 0x61, 0x1e, 0xe2, 0x10, 0x60, 0x9e,
      0x94, 0x88, 0xd5, 0xa8, 0x9d, 0xe6, 0xf5, 0x73,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_pinky_interfaces
const rosidl_type_hash_t *
pinky_interfaces__srv__SetLamp_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x6c, 0xfb, 0x81, 0xb7, 0xc5, 0x1f, 0x41, 0x87,
      0x2b, 0x59, 0xf1, 0x24, 0xce, 0x24, 0x71, 0x86,
      0xd8, 0xc5, 0x79, 0x64, 0x74, 0xde, 0x43, 0x5b,
      0x23, 0x27, 0x1c, 0x41, 0x41, 0xe4, 0x33, 0x67,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "service_msgs/msg/detail/service_event_info__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "std_msgs/msg/detail/color_rgba__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
static const rosidl_type_hash_t std_msgs__msg__ColorRGBA__EXPECTED_HASH = {1, {
    0x77, 0xa7, 0xa5, 0xb9, 0xae, 0x47, 0x73, 0x06,
    0x09, 0x76, 0x65, 0x10, 0x6e, 0x04, 0x13, 0xba,
    0x74, 0x44, 0x02, 0x45, 0xb1, 0xf3, 0xd0, 0xc6,
    0xd6, 0x40, 0x5f, 0xe5, 0xc7, 0x81, 0x3f, 0xe8,
  }};
#endif

static char pinky_interfaces__srv__SetLamp__TYPE_NAME[] = "pinky_interfaces/srv/SetLamp";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char pinky_interfaces__srv__SetLamp_Event__TYPE_NAME[] = "pinky_interfaces/srv/SetLamp_Event";
static char pinky_interfaces__srv__SetLamp_Request__TYPE_NAME[] = "pinky_interfaces/srv/SetLamp_Request";
static char pinky_interfaces__srv__SetLamp_Response__TYPE_NAME[] = "pinky_interfaces/srv/SetLamp_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";
static char std_msgs__msg__ColorRGBA__TYPE_NAME[] = "std_msgs/msg/ColorRGBA";

// Define type names, field names, and default values
static char pinky_interfaces__srv__SetLamp__FIELD_NAME__request_message[] = "request_message";
static char pinky_interfaces__srv__SetLamp__FIELD_NAME__response_message[] = "response_message";
static char pinky_interfaces__srv__SetLamp__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field pinky_interfaces__srv__SetLamp__FIELDS[] = {
  {
    {pinky_interfaces__srv__SetLamp__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {pinky_interfaces__srv__SetLamp_Request__TYPE_NAME, 36, 36},
    },
    {NULL, 0, 0},
  },
  {
    {pinky_interfaces__srv__SetLamp__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {pinky_interfaces__srv__SetLamp_Response__TYPE_NAME, 37, 37},
    },
    {NULL, 0, 0},
  },
  {
    {pinky_interfaces__srv__SetLamp__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {pinky_interfaces__srv__SetLamp_Event__TYPE_NAME, 34, 34},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription pinky_interfaces__srv__SetLamp__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {pinky_interfaces__srv__SetLamp_Event__TYPE_NAME, 34, 34},
    {NULL, 0, 0},
  },
  {
    {pinky_interfaces__srv__SetLamp_Request__TYPE_NAME, 36, 36},
    {NULL, 0, 0},
  },
  {
    {pinky_interfaces__srv__SetLamp_Response__TYPE_NAME, 37, 37},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__ColorRGBA__TYPE_NAME, 22, 22},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
pinky_interfaces__srv__SetLamp__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {pinky_interfaces__srv__SetLamp__TYPE_NAME, 28, 28},
      {pinky_interfaces__srv__SetLamp__FIELDS, 3, 3},
    },
    {pinky_interfaces__srv__SetLamp__REFERENCED_TYPE_DESCRIPTIONS, 6, 6},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = pinky_interfaces__srv__SetLamp_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = pinky_interfaces__srv__SetLamp_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = pinky_interfaces__srv__SetLamp_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__ColorRGBA__EXPECTED_HASH, std_msgs__msg__ColorRGBA__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[5].fields = std_msgs__msg__ColorRGBA__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char pinky_interfaces__srv__SetLamp_Request__FIELD_NAME__color[] = "color";
static char pinky_interfaces__srv__SetLamp_Request__FIELD_NAME__mode[] = "mode";
static char pinky_interfaces__srv__SetLamp_Request__FIELD_NAME__time[] = "time";

static rosidl_runtime_c__type_description__Field pinky_interfaces__srv__SetLamp_Request__FIELDS[] = {
  {
    {pinky_interfaces__srv__SetLamp_Request__FIELD_NAME__color, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__ColorRGBA__TYPE_NAME, 22, 22},
    },
    {NULL, 0, 0},
  },
  {
    {pinky_interfaces__srv__SetLamp_Request__FIELD_NAME__mode, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pinky_interfaces__srv__SetLamp_Request__FIELD_NAME__time, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription pinky_interfaces__srv__SetLamp_Request__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {std_msgs__msg__ColorRGBA__TYPE_NAME, 22, 22},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
pinky_interfaces__srv__SetLamp_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {pinky_interfaces__srv__SetLamp_Request__TYPE_NAME, 36, 36},
      {pinky_interfaces__srv__SetLamp_Request__FIELDS, 3, 3},
    },
    {pinky_interfaces__srv__SetLamp_Request__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&std_msgs__msg__ColorRGBA__EXPECTED_HASH, std_msgs__msg__ColorRGBA__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = std_msgs__msg__ColorRGBA__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char pinky_interfaces__srv__SetLamp_Response__FIELD_NAME__result[] = "result";

static rosidl_runtime_c__type_description__Field pinky_interfaces__srv__SetLamp_Response__FIELDS[] = {
  {
    {pinky_interfaces__srv__SetLamp_Response__FIELD_NAME__result, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
pinky_interfaces__srv__SetLamp_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {pinky_interfaces__srv__SetLamp_Response__TYPE_NAME, 37, 37},
      {pinky_interfaces__srv__SetLamp_Response__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char pinky_interfaces__srv__SetLamp_Event__FIELD_NAME__info[] = "info";
static char pinky_interfaces__srv__SetLamp_Event__FIELD_NAME__request[] = "request";
static char pinky_interfaces__srv__SetLamp_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field pinky_interfaces__srv__SetLamp_Event__FIELDS[] = {
  {
    {pinky_interfaces__srv__SetLamp_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {pinky_interfaces__srv__SetLamp_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {pinky_interfaces__srv__SetLamp_Request__TYPE_NAME, 36, 36},
    },
    {NULL, 0, 0},
  },
  {
    {pinky_interfaces__srv__SetLamp_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {pinky_interfaces__srv__SetLamp_Response__TYPE_NAME, 37, 37},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription pinky_interfaces__srv__SetLamp_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {pinky_interfaces__srv__SetLamp_Request__TYPE_NAME, 36, 36},
    {NULL, 0, 0},
  },
  {
    {pinky_interfaces__srv__SetLamp_Response__TYPE_NAME, 37, 37},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__ColorRGBA__TYPE_NAME, 22, 22},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
pinky_interfaces__srv__SetLamp_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {pinky_interfaces__srv__SetLamp_Event__TYPE_NAME, 34, 34},
      {pinky_interfaces__srv__SetLamp_Event__FIELDS, 3, 3},
    },
    {pinky_interfaces__srv__SetLamp_Event__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = pinky_interfaces__srv__SetLamp_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = pinky_interfaces__srv__SetLamp_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__ColorRGBA__EXPECTED_HASH, std_msgs__msg__ColorRGBA__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = std_msgs__msg__ColorRGBA__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "std_msgs/ColorRGBA color\n"
  "uint8 mode\n"
  "uint16 time\n"
  "---\n"
  "bool result";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
pinky_interfaces__srv__SetLamp__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {pinky_interfaces__srv__SetLamp__TYPE_NAME, 28, 28},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 63, 63},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
pinky_interfaces__srv__SetLamp_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {pinky_interfaces__srv__SetLamp_Request__TYPE_NAME, 36, 36},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
pinky_interfaces__srv__SetLamp_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {pinky_interfaces__srv__SetLamp_Response__TYPE_NAME, 37, 37},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
pinky_interfaces__srv__SetLamp_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {pinky_interfaces__srv__SetLamp_Event__TYPE_NAME, 34, 34},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
pinky_interfaces__srv__SetLamp__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[7];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 7, 7};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *pinky_interfaces__srv__SetLamp__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *pinky_interfaces__srv__SetLamp_Event__get_individual_type_description_source(NULL);
    sources[3] = *pinky_interfaces__srv__SetLamp_Request__get_individual_type_description_source(NULL);
    sources[4] = *pinky_interfaces__srv__SetLamp_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[6] = *std_msgs__msg__ColorRGBA__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
pinky_interfaces__srv__SetLamp_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *pinky_interfaces__srv__SetLamp_Request__get_individual_type_description_source(NULL),
    sources[1] = *std_msgs__msg__ColorRGBA__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
pinky_interfaces__srv__SetLamp_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *pinky_interfaces__srv__SetLamp_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
pinky_interfaces__srv__SetLamp_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *pinky_interfaces__srv__SetLamp_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *pinky_interfaces__srv__SetLamp_Request__get_individual_type_description_source(NULL);
    sources[3] = *pinky_interfaces__srv__SetLamp_Response__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[5] = *std_msgs__msg__ColorRGBA__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
