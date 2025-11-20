// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from pinky_interfaces:srv/SetBrightness.idl
// generated code does not contain a copyright notice

#include "pinky_interfaces/srv/detail/set_brightness__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_pinky_interfaces
const rosidl_type_hash_t *
pinky_interfaces__srv__SetBrightness__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x57, 0xa0, 0x4c, 0x07, 0x68, 0xfa, 0xd0, 0x3e,
      0x46, 0x37, 0x29, 0x7e, 0x09, 0x4a, 0xcd, 0xf4,
      0x7e, 0x40, 0xcf, 0x8f, 0xe1, 0xe3, 0x9a, 0x46,
      0x68, 0x38, 0x57, 0xcd, 0x8a, 0x68, 0x1a, 0xf2,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_pinky_interfaces
const rosidl_type_hash_t *
pinky_interfaces__srv__SetBrightness_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x62, 0xa5, 0xa0, 0xf9, 0xf7, 0x56, 0x5b, 0x1c,
      0xe7, 0x53, 0xeb, 0xe1, 0x82, 0xae, 0x1a, 0xbb,
      0xc5, 0xc4, 0x8c, 0xd4, 0x88, 0xd9, 0x2a, 0x2d,
      0x88, 0x02, 0x9c, 0xaa, 0x40, 0x2f, 0x61, 0x4e,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_pinky_interfaces
const rosidl_type_hash_t *
pinky_interfaces__srv__SetBrightness_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x49, 0x42, 0x6d, 0xae, 0x8e, 0x5e, 0x87, 0xd5,
      0x0a, 0xf4, 0xbf, 0x5d, 0x29, 0xa9, 0xa4, 0x9a,
      0xdb, 0x28, 0x0d, 0x18, 0xe9, 0x10, 0x0f, 0xbb,
      0x6b, 0x25, 0x2c, 0x42, 0x14, 0x77, 0x23, 0x66,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_pinky_interfaces
const rosidl_type_hash_t *
pinky_interfaces__srv__SetBrightness_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x1e, 0xe9, 0x7b, 0xeb, 0xba, 0x7a, 0x31, 0x84,
      0xa8, 0x70, 0xdf, 0x20, 0x48, 0xe4, 0x7a, 0xe6,
      0x4d, 0x58, 0xaf, 0xe7, 0x92, 0x6f, 0xe9, 0x57,
      0xf6, 0x1e, 0x8a, 0x99, 0x4b, 0x34, 0x3e, 0x95,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "service_msgs/msg/detail/service_event_info__functions.h"

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
#endif

static char pinky_interfaces__srv__SetBrightness__TYPE_NAME[] = "pinky_interfaces/srv/SetBrightness";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char pinky_interfaces__srv__SetBrightness_Event__TYPE_NAME[] = "pinky_interfaces/srv/SetBrightness_Event";
static char pinky_interfaces__srv__SetBrightness_Request__TYPE_NAME[] = "pinky_interfaces/srv/SetBrightness_Request";
static char pinky_interfaces__srv__SetBrightness_Response__TYPE_NAME[] = "pinky_interfaces/srv/SetBrightness_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char pinky_interfaces__srv__SetBrightness__FIELD_NAME__request_message[] = "request_message";
static char pinky_interfaces__srv__SetBrightness__FIELD_NAME__response_message[] = "response_message";
static char pinky_interfaces__srv__SetBrightness__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field pinky_interfaces__srv__SetBrightness__FIELDS[] = {
  {
    {pinky_interfaces__srv__SetBrightness__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {pinky_interfaces__srv__SetBrightness_Request__TYPE_NAME, 42, 42},
    },
    {NULL, 0, 0},
  },
  {
    {pinky_interfaces__srv__SetBrightness__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {pinky_interfaces__srv__SetBrightness_Response__TYPE_NAME, 43, 43},
    },
    {NULL, 0, 0},
  },
  {
    {pinky_interfaces__srv__SetBrightness__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {pinky_interfaces__srv__SetBrightness_Event__TYPE_NAME, 40, 40},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription pinky_interfaces__srv__SetBrightness__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {pinky_interfaces__srv__SetBrightness_Event__TYPE_NAME, 40, 40},
    {NULL, 0, 0},
  },
  {
    {pinky_interfaces__srv__SetBrightness_Request__TYPE_NAME, 42, 42},
    {NULL, 0, 0},
  },
  {
    {pinky_interfaces__srv__SetBrightness_Response__TYPE_NAME, 43, 43},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
pinky_interfaces__srv__SetBrightness__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {pinky_interfaces__srv__SetBrightness__TYPE_NAME, 34, 34},
      {pinky_interfaces__srv__SetBrightness__FIELDS, 3, 3},
    },
    {pinky_interfaces__srv__SetBrightness__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = pinky_interfaces__srv__SetBrightness_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = pinky_interfaces__srv__SetBrightness_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = pinky_interfaces__srv__SetBrightness_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char pinky_interfaces__srv__SetBrightness_Request__FIELD_NAME__brightness[] = "brightness";

static rosidl_runtime_c__type_description__Field pinky_interfaces__srv__SetBrightness_Request__FIELDS[] = {
  {
    {pinky_interfaces__srv__SetBrightness_Request__FIELD_NAME__brightness, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
pinky_interfaces__srv__SetBrightness_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {pinky_interfaces__srv__SetBrightness_Request__TYPE_NAME, 42, 42},
      {pinky_interfaces__srv__SetBrightness_Request__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char pinky_interfaces__srv__SetBrightness_Response__FIELD_NAME__success[] = "success";
static char pinky_interfaces__srv__SetBrightness_Response__FIELD_NAME__message[] = "message";

static rosidl_runtime_c__type_description__Field pinky_interfaces__srv__SetBrightness_Response__FIELDS[] = {
  {
    {pinky_interfaces__srv__SetBrightness_Response__FIELD_NAME__success, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pinky_interfaces__srv__SetBrightness_Response__FIELD_NAME__message, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
pinky_interfaces__srv__SetBrightness_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {pinky_interfaces__srv__SetBrightness_Response__TYPE_NAME, 43, 43},
      {pinky_interfaces__srv__SetBrightness_Response__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char pinky_interfaces__srv__SetBrightness_Event__FIELD_NAME__info[] = "info";
static char pinky_interfaces__srv__SetBrightness_Event__FIELD_NAME__request[] = "request";
static char pinky_interfaces__srv__SetBrightness_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field pinky_interfaces__srv__SetBrightness_Event__FIELDS[] = {
  {
    {pinky_interfaces__srv__SetBrightness_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {pinky_interfaces__srv__SetBrightness_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {pinky_interfaces__srv__SetBrightness_Request__TYPE_NAME, 42, 42},
    },
    {NULL, 0, 0},
  },
  {
    {pinky_interfaces__srv__SetBrightness_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {pinky_interfaces__srv__SetBrightness_Response__TYPE_NAME, 43, 43},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription pinky_interfaces__srv__SetBrightness_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {pinky_interfaces__srv__SetBrightness_Request__TYPE_NAME, 42, 42},
    {NULL, 0, 0},
  },
  {
    {pinky_interfaces__srv__SetBrightness_Response__TYPE_NAME, 43, 43},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
pinky_interfaces__srv__SetBrightness_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {pinky_interfaces__srv__SetBrightness_Event__TYPE_NAME, 40, 40},
      {pinky_interfaces__srv__SetBrightness_Event__FIELDS, 3, 3},
    },
    {pinky_interfaces__srv__SetBrightness_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = pinky_interfaces__srv__SetBrightness_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = pinky_interfaces__srv__SetBrightness_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "int32 brightness\n"
  "---\n"
  "bool success\n"
  "string message";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
pinky_interfaces__srv__SetBrightness__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {pinky_interfaces__srv__SetBrightness__TYPE_NAME, 34, 34},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 48, 48},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
pinky_interfaces__srv__SetBrightness_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {pinky_interfaces__srv__SetBrightness_Request__TYPE_NAME, 42, 42},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
pinky_interfaces__srv__SetBrightness_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {pinky_interfaces__srv__SetBrightness_Response__TYPE_NAME, 43, 43},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
pinky_interfaces__srv__SetBrightness_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {pinky_interfaces__srv__SetBrightness_Event__TYPE_NAME, 40, 40},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
pinky_interfaces__srv__SetBrightness__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *pinky_interfaces__srv__SetBrightness__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *pinky_interfaces__srv__SetBrightness_Event__get_individual_type_description_source(NULL);
    sources[3] = *pinky_interfaces__srv__SetBrightness_Request__get_individual_type_description_source(NULL);
    sources[4] = *pinky_interfaces__srv__SetBrightness_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
pinky_interfaces__srv__SetBrightness_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *pinky_interfaces__srv__SetBrightness_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
pinky_interfaces__srv__SetBrightness_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *pinky_interfaces__srv__SetBrightness_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
pinky_interfaces__srv__SetBrightness_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *pinky_interfaces__srv__SetBrightness_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *pinky_interfaces__srv__SetBrightness_Request__get_individual_type_description_source(NULL);
    sources[3] = *pinky_interfaces__srv__SetBrightness_Response__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
