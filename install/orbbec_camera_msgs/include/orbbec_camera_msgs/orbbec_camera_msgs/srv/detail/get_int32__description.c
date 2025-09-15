// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from orbbec_camera_msgs:srv/GetInt32.idl
// generated code does not contain a copyright notice

#include "orbbec_camera_msgs/srv/detail/get_int32__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_orbbec_camera_msgs
const rosidl_type_hash_t *
orbbec_camera_msgs__srv__GetInt32__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xdb, 0xed, 0xf5, 0x6c, 0x78, 0xf3, 0x38, 0xd7,
      0xfb, 0x5d, 0x88, 0x21, 0xb7, 0x3c, 0xd2, 0xb1,
      0x75, 0x46, 0x08, 0xf3, 0xf6, 0x8d, 0x9d, 0x3a,
      0x46, 0xcb, 0x2c, 0x33, 0xdb, 0x05, 0x78, 0x18,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_orbbec_camera_msgs
const rosidl_type_hash_t *
orbbec_camera_msgs__srv__GetInt32_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x27, 0xdd, 0x2d, 0x1f, 0xe3, 0xc4, 0x18, 0x4b,
      0x2f, 0x92, 0xe9, 0x7d, 0x6d, 0xcb, 0xb2, 0x03,
      0xb0, 0x15, 0xbd, 0x70, 0xd4, 0x5b, 0x94, 0x33,
      0x0d, 0x58, 0x9e, 0x4c, 0xfc, 0x08, 0x19, 0x4e,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_orbbec_camera_msgs
const rosidl_type_hash_t *
orbbec_camera_msgs__srv__GetInt32_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x1f, 0xbd, 0x2f, 0x14, 0x97, 0x37, 0xc3, 0xb7,
      0x82, 0xdd, 0x6b, 0x1b, 0x91, 0x23, 0x15, 0x90,
      0xcb, 0x9a, 0xa9, 0x64, 0x35, 0x6d, 0x1c, 0xeb,
      0x4d, 0xeb, 0x1a, 0xa9, 0x6c, 0x95, 0x58, 0x13,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_orbbec_camera_msgs
const rosidl_type_hash_t *
orbbec_camera_msgs__srv__GetInt32_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xbd, 0x23, 0x15, 0x2a, 0x54, 0xf7, 0x6a, 0x53,
      0x3a, 0x11, 0x4f, 0x80, 0xef, 0x97, 0x15, 0x69,
      0xd5, 0x14, 0x37, 0x57, 0xf1, 0xd8, 0x4f, 0xfa,
      0xc1, 0x2b, 0xef, 0x64, 0x2a, 0x36, 0xd0, 0x7a,
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

static char orbbec_camera_msgs__srv__GetInt32__TYPE_NAME[] = "orbbec_camera_msgs/srv/GetInt32";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char orbbec_camera_msgs__srv__GetInt32_Event__TYPE_NAME[] = "orbbec_camera_msgs/srv/GetInt32_Event";
static char orbbec_camera_msgs__srv__GetInt32_Request__TYPE_NAME[] = "orbbec_camera_msgs/srv/GetInt32_Request";
static char orbbec_camera_msgs__srv__GetInt32_Response__TYPE_NAME[] = "orbbec_camera_msgs/srv/GetInt32_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char orbbec_camera_msgs__srv__GetInt32__FIELD_NAME__request_message[] = "request_message";
static char orbbec_camera_msgs__srv__GetInt32__FIELD_NAME__response_message[] = "response_message";
static char orbbec_camera_msgs__srv__GetInt32__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field orbbec_camera_msgs__srv__GetInt32__FIELDS[] = {
  {
    {orbbec_camera_msgs__srv__GetInt32__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {orbbec_camera_msgs__srv__GetInt32_Request__TYPE_NAME, 39, 39},
    },
    {NULL, 0, 0},
  },
  {
    {orbbec_camera_msgs__srv__GetInt32__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {orbbec_camera_msgs__srv__GetInt32_Response__TYPE_NAME, 40, 40},
    },
    {NULL, 0, 0},
  },
  {
    {orbbec_camera_msgs__srv__GetInt32__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {orbbec_camera_msgs__srv__GetInt32_Event__TYPE_NAME, 37, 37},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription orbbec_camera_msgs__srv__GetInt32__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {orbbec_camera_msgs__srv__GetInt32_Event__TYPE_NAME, 37, 37},
    {NULL, 0, 0},
  },
  {
    {orbbec_camera_msgs__srv__GetInt32_Request__TYPE_NAME, 39, 39},
    {NULL, 0, 0},
  },
  {
    {orbbec_camera_msgs__srv__GetInt32_Response__TYPE_NAME, 40, 40},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
orbbec_camera_msgs__srv__GetInt32__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {orbbec_camera_msgs__srv__GetInt32__TYPE_NAME, 31, 31},
      {orbbec_camera_msgs__srv__GetInt32__FIELDS, 3, 3},
    },
    {orbbec_camera_msgs__srv__GetInt32__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = orbbec_camera_msgs__srv__GetInt32_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = orbbec_camera_msgs__srv__GetInt32_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = orbbec_camera_msgs__srv__GetInt32_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char orbbec_camera_msgs__srv__GetInt32_Request__FIELD_NAME__structure_needs_at_least_one_member[] = "structure_needs_at_least_one_member";

static rosidl_runtime_c__type_description__Field orbbec_camera_msgs__srv__GetInt32_Request__FIELDS[] = {
  {
    {orbbec_camera_msgs__srv__GetInt32_Request__FIELD_NAME__structure_needs_at_least_one_member, 35, 35},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
orbbec_camera_msgs__srv__GetInt32_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {orbbec_camera_msgs__srv__GetInt32_Request__TYPE_NAME, 39, 39},
      {orbbec_camera_msgs__srv__GetInt32_Request__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char orbbec_camera_msgs__srv__GetInt32_Response__FIELD_NAME__data[] = "data";
static char orbbec_camera_msgs__srv__GetInt32_Response__FIELD_NAME__success[] = "success";
static char orbbec_camera_msgs__srv__GetInt32_Response__FIELD_NAME__message[] = "message";

static rosidl_runtime_c__type_description__Field orbbec_camera_msgs__srv__GetInt32_Response__FIELDS[] = {
  {
    {orbbec_camera_msgs__srv__GetInt32_Response__FIELD_NAME__data, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {orbbec_camera_msgs__srv__GetInt32_Response__FIELD_NAME__success, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {orbbec_camera_msgs__srv__GetInt32_Response__FIELD_NAME__message, 7, 7},
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
orbbec_camera_msgs__srv__GetInt32_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {orbbec_camera_msgs__srv__GetInt32_Response__TYPE_NAME, 40, 40},
      {orbbec_camera_msgs__srv__GetInt32_Response__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char orbbec_camera_msgs__srv__GetInt32_Event__FIELD_NAME__info[] = "info";
static char orbbec_camera_msgs__srv__GetInt32_Event__FIELD_NAME__request[] = "request";
static char orbbec_camera_msgs__srv__GetInt32_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field orbbec_camera_msgs__srv__GetInt32_Event__FIELDS[] = {
  {
    {orbbec_camera_msgs__srv__GetInt32_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {orbbec_camera_msgs__srv__GetInt32_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {orbbec_camera_msgs__srv__GetInt32_Request__TYPE_NAME, 39, 39},
    },
    {NULL, 0, 0},
  },
  {
    {orbbec_camera_msgs__srv__GetInt32_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {orbbec_camera_msgs__srv__GetInt32_Response__TYPE_NAME, 40, 40},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription orbbec_camera_msgs__srv__GetInt32_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {orbbec_camera_msgs__srv__GetInt32_Request__TYPE_NAME, 39, 39},
    {NULL, 0, 0},
  },
  {
    {orbbec_camera_msgs__srv__GetInt32_Response__TYPE_NAME, 40, 40},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
orbbec_camera_msgs__srv__GetInt32_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {orbbec_camera_msgs__srv__GetInt32_Event__TYPE_NAME, 37, 37},
      {orbbec_camera_msgs__srv__GetInt32_Event__FIELDS, 3, 3},
    },
    {orbbec_camera_msgs__srv__GetInt32_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = orbbec_camera_msgs__srv__GetInt32_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = orbbec_camera_msgs__srv__GetInt32_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "---\n"
  "int32 data\n"
  "bool success\n"
  "string message";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
orbbec_camera_msgs__srv__GetInt32__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {orbbec_camera_msgs__srv__GetInt32__TYPE_NAME, 31, 31},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 43, 43},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
orbbec_camera_msgs__srv__GetInt32_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {orbbec_camera_msgs__srv__GetInt32_Request__TYPE_NAME, 39, 39},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
orbbec_camera_msgs__srv__GetInt32_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {orbbec_camera_msgs__srv__GetInt32_Response__TYPE_NAME, 40, 40},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
orbbec_camera_msgs__srv__GetInt32_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {orbbec_camera_msgs__srv__GetInt32_Event__TYPE_NAME, 37, 37},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
orbbec_camera_msgs__srv__GetInt32__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *orbbec_camera_msgs__srv__GetInt32__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *orbbec_camera_msgs__srv__GetInt32_Event__get_individual_type_description_source(NULL);
    sources[3] = *orbbec_camera_msgs__srv__GetInt32_Request__get_individual_type_description_source(NULL);
    sources[4] = *orbbec_camera_msgs__srv__GetInt32_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
orbbec_camera_msgs__srv__GetInt32_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *orbbec_camera_msgs__srv__GetInt32_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
orbbec_camera_msgs__srv__GetInt32_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *orbbec_camera_msgs__srv__GetInt32_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
orbbec_camera_msgs__srv__GetInt32_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *orbbec_camera_msgs__srv__GetInt32_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *orbbec_camera_msgs__srv__GetInt32_Request__get_individual_type_description_source(NULL);
    sources[3] = *orbbec_camera_msgs__srv__GetInt32_Response__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
