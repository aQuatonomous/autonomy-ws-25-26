// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from pointcloud_filters:srv/GetCloudWindow.idl
// generated code does not contain a copyright notice
#include "pointcloud_filters/srv/detail/get_cloud_window__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
pointcloud_filters__srv__GetCloudWindow_Request__init(pointcloud_filters__srv__GetCloudWindow_Request * msg)
{
  if (!msg) {
    return false;
  }
  // window_sec
  // merged
  return true;
}

void
pointcloud_filters__srv__GetCloudWindow_Request__fini(pointcloud_filters__srv__GetCloudWindow_Request * msg)
{
  if (!msg) {
    return;
  }
  // window_sec
  // merged
}

bool
pointcloud_filters__srv__GetCloudWindow_Request__are_equal(const pointcloud_filters__srv__GetCloudWindow_Request * lhs, const pointcloud_filters__srv__GetCloudWindow_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // window_sec
  if (lhs->window_sec != rhs->window_sec) {
    return false;
  }
  // merged
  if (lhs->merged != rhs->merged) {
    return false;
  }
  return true;
}

bool
pointcloud_filters__srv__GetCloudWindow_Request__copy(
  const pointcloud_filters__srv__GetCloudWindow_Request * input,
  pointcloud_filters__srv__GetCloudWindow_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // window_sec
  output->window_sec = input->window_sec;
  // merged
  output->merged = input->merged;
  return true;
}

pointcloud_filters__srv__GetCloudWindow_Request *
pointcloud_filters__srv__GetCloudWindow_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pointcloud_filters__srv__GetCloudWindow_Request * msg = (pointcloud_filters__srv__GetCloudWindow_Request *)allocator.allocate(sizeof(pointcloud_filters__srv__GetCloudWindow_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(pointcloud_filters__srv__GetCloudWindow_Request));
  bool success = pointcloud_filters__srv__GetCloudWindow_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
pointcloud_filters__srv__GetCloudWindow_Request__destroy(pointcloud_filters__srv__GetCloudWindow_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    pointcloud_filters__srv__GetCloudWindow_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
pointcloud_filters__srv__GetCloudWindow_Request__Sequence__init(pointcloud_filters__srv__GetCloudWindow_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pointcloud_filters__srv__GetCloudWindow_Request * data = NULL;

  if (size) {
    data = (pointcloud_filters__srv__GetCloudWindow_Request *)allocator.zero_allocate(size, sizeof(pointcloud_filters__srv__GetCloudWindow_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = pointcloud_filters__srv__GetCloudWindow_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        pointcloud_filters__srv__GetCloudWindow_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
pointcloud_filters__srv__GetCloudWindow_Request__Sequence__fini(pointcloud_filters__srv__GetCloudWindow_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      pointcloud_filters__srv__GetCloudWindow_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

pointcloud_filters__srv__GetCloudWindow_Request__Sequence *
pointcloud_filters__srv__GetCloudWindow_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pointcloud_filters__srv__GetCloudWindow_Request__Sequence * array = (pointcloud_filters__srv__GetCloudWindow_Request__Sequence *)allocator.allocate(sizeof(pointcloud_filters__srv__GetCloudWindow_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = pointcloud_filters__srv__GetCloudWindow_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
pointcloud_filters__srv__GetCloudWindow_Request__Sequence__destroy(pointcloud_filters__srv__GetCloudWindow_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    pointcloud_filters__srv__GetCloudWindow_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
pointcloud_filters__srv__GetCloudWindow_Request__Sequence__are_equal(const pointcloud_filters__srv__GetCloudWindow_Request__Sequence * lhs, const pointcloud_filters__srv__GetCloudWindow_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!pointcloud_filters__srv__GetCloudWindow_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
pointcloud_filters__srv__GetCloudWindow_Request__Sequence__copy(
  const pointcloud_filters__srv__GetCloudWindow_Request__Sequence * input,
  pointcloud_filters__srv__GetCloudWindow_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(pointcloud_filters__srv__GetCloudWindow_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    pointcloud_filters__srv__GetCloudWindow_Request * data =
      (pointcloud_filters__srv__GetCloudWindow_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!pointcloud_filters__srv__GetCloudWindow_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          pointcloud_filters__srv__GetCloudWindow_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!pointcloud_filters__srv__GetCloudWindow_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `clouds`
// Member `merged_cloud`
#include "sensor_msgs/msg/detail/point_cloud2__functions.h"

bool
pointcloud_filters__srv__GetCloudWindow_Response__init(pointcloud_filters__srv__GetCloudWindow_Response * msg)
{
  if (!msg) {
    return false;
  }
  // clouds
  if (!sensor_msgs__msg__PointCloud2__Sequence__init(&msg->clouds, 0)) {
    pointcloud_filters__srv__GetCloudWindow_Response__fini(msg);
    return false;
  }
  // merged_cloud
  if (!sensor_msgs__msg__PointCloud2__init(&msg->merged_cloud)) {
    pointcloud_filters__srv__GetCloudWindow_Response__fini(msg);
    return false;
  }
  return true;
}

void
pointcloud_filters__srv__GetCloudWindow_Response__fini(pointcloud_filters__srv__GetCloudWindow_Response * msg)
{
  if (!msg) {
    return;
  }
  // clouds
  sensor_msgs__msg__PointCloud2__Sequence__fini(&msg->clouds);
  // merged_cloud
  sensor_msgs__msg__PointCloud2__fini(&msg->merged_cloud);
}

bool
pointcloud_filters__srv__GetCloudWindow_Response__are_equal(const pointcloud_filters__srv__GetCloudWindow_Response * lhs, const pointcloud_filters__srv__GetCloudWindow_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // clouds
  if (!sensor_msgs__msg__PointCloud2__Sequence__are_equal(
      &(lhs->clouds), &(rhs->clouds)))
  {
    return false;
  }
  // merged_cloud
  if (!sensor_msgs__msg__PointCloud2__are_equal(
      &(lhs->merged_cloud), &(rhs->merged_cloud)))
  {
    return false;
  }
  return true;
}

bool
pointcloud_filters__srv__GetCloudWindow_Response__copy(
  const pointcloud_filters__srv__GetCloudWindow_Response * input,
  pointcloud_filters__srv__GetCloudWindow_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // clouds
  if (!sensor_msgs__msg__PointCloud2__Sequence__copy(
      &(input->clouds), &(output->clouds)))
  {
    return false;
  }
  // merged_cloud
  if (!sensor_msgs__msg__PointCloud2__copy(
      &(input->merged_cloud), &(output->merged_cloud)))
  {
    return false;
  }
  return true;
}

pointcloud_filters__srv__GetCloudWindow_Response *
pointcloud_filters__srv__GetCloudWindow_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pointcloud_filters__srv__GetCloudWindow_Response * msg = (pointcloud_filters__srv__GetCloudWindow_Response *)allocator.allocate(sizeof(pointcloud_filters__srv__GetCloudWindow_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(pointcloud_filters__srv__GetCloudWindow_Response));
  bool success = pointcloud_filters__srv__GetCloudWindow_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
pointcloud_filters__srv__GetCloudWindow_Response__destroy(pointcloud_filters__srv__GetCloudWindow_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    pointcloud_filters__srv__GetCloudWindow_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
pointcloud_filters__srv__GetCloudWindow_Response__Sequence__init(pointcloud_filters__srv__GetCloudWindow_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pointcloud_filters__srv__GetCloudWindow_Response * data = NULL;

  if (size) {
    data = (pointcloud_filters__srv__GetCloudWindow_Response *)allocator.zero_allocate(size, sizeof(pointcloud_filters__srv__GetCloudWindow_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = pointcloud_filters__srv__GetCloudWindow_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        pointcloud_filters__srv__GetCloudWindow_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
pointcloud_filters__srv__GetCloudWindow_Response__Sequence__fini(pointcloud_filters__srv__GetCloudWindow_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      pointcloud_filters__srv__GetCloudWindow_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

pointcloud_filters__srv__GetCloudWindow_Response__Sequence *
pointcloud_filters__srv__GetCloudWindow_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pointcloud_filters__srv__GetCloudWindow_Response__Sequence * array = (pointcloud_filters__srv__GetCloudWindow_Response__Sequence *)allocator.allocate(sizeof(pointcloud_filters__srv__GetCloudWindow_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = pointcloud_filters__srv__GetCloudWindow_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
pointcloud_filters__srv__GetCloudWindow_Response__Sequence__destroy(pointcloud_filters__srv__GetCloudWindow_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    pointcloud_filters__srv__GetCloudWindow_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
pointcloud_filters__srv__GetCloudWindow_Response__Sequence__are_equal(const pointcloud_filters__srv__GetCloudWindow_Response__Sequence * lhs, const pointcloud_filters__srv__GetCloudWindow_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!pointcloud_filters__srv__GetCloudWindow_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
pointcloud_filters__srv__GetCloudWindow_Response__Sequence__copy(
  const pointcloud_filters__srv__GetCloudWindow_Response__Sequence * input,
  pointcloud_filters__srv__GetCloudWindow_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(pointcloud_filters__srv__GetCloudWindow_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    pointcloud_filters__srv__GetCloudWindow_Response * data =
      (pointcloud_filters__srv__GetCloudWindow_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!pointcloud_filters__srv__GetCloudWindow_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          pointcloud_filters__srv__GetCloudWindow_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!pointcloud_filters__srv__GetCloudWindow_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
