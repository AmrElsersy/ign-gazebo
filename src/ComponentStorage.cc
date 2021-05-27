/*
 * Copyright (C) 2021 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include "ignition/gazebo/detail/ComponentStorage.hh"

#include <memory>
#include <utility>

#include "ignition/gazebo/Entity.hh"
#include "ignition/gazebo/Types.hh"

using namespace ignition;
using namespace gazebo;
using namespace detail;

//////////////////////////////////////////////////
void ComponentStorage::Reset()
{
  this->entityComponents.clear();
  this->componentTypeIndex.clear();
}

//////////////////////////////////////////////////
bool ComponentStorage::AddEntity(const Entity _entity)
{
  if (this->entityComponents.find(_entity) != this->entityComponents.end())
    return false;
  this->entityComponents[_entity];

  if (this->componentTypeIndex.find(_entity) != this->componentTypeIndex.end())
    return false;
  this->componentTypeIndex[_entity];

  return true;
}

//////////////////////////////////////////////////
bool ComponentStorage::RemoveEntity(const Entity _entity)
{
  const auto removedComponents = this->entityComponents.erase(_entity);
  const auto removedTypeIdxMap = this->componentTypeIndex.erase(_entity);
  return removedComponents && removedTypeIdxMap;
}

//////////////////////////////////////////////////
ComponentAdditionResult ComponentStorage::AddComponent(
    const Entity _entity,
    std::unique_ptr<components::BaseComponent> _component)
{
  // TODO refactor this method along with the templated version to reduce
  // duplicate code? Perhaps the strategy pattern would be useful here

  // make sure the entity exists
  auto typeMapIter = this->componentTypeIndex.find(_entity);
  auto entityCompIter = this->entityComponents.find(_entity);
  if (typeMapIter == this->componentTypeIndex.end() ||
      entityCompIter == this->entityComponents.end())
    return ComponentAdditionResult::FAILED_ADDITION;

  const auto typeId = _component->TypeId();

  // get the pre-existing instance to the entity's component of the same type,
  // if it exists
  auto existingCompPtr = this->Component(_entity, typeId);
  if (nullptr == existingCompPtr)
  {
    // the component to be added is brand new to the entity
    const auto vectorIdx = entityCompIter->second.size();
    entityCompIter->second.push_back(std::move(_component));
    this->componentTypeIndex[_entity][typeId] = vectorIdx;
    return ComponentAdditionResult::NEW_ADDITION;
  }

  // if the pre-existing component is being ignored, this means that the
  // component was added to the entity previously, but later removed. In this
  // case, a re-addition of the component is occuring. If the pre-existing
  // component is not being ignored, this means that the component was added
  // to the entity previously and never removed. In this case, we are simply
  // modifying the data of the pre-existing component (the modification of the
  // data is done externally in a templated ECM method call, because we need the
  // derived component class in order to update the derived component data)
  const auto additionResult = existingCompPtr->ignore ?
    ComponentAdditionResult::RE_ADDITION :
    ComponentAdditionResult::MODIFICATION;
  existingCompPtr->ignore = false;
  return additionResult;
}

//////////////////////////////////////////////////
bool ComponentStorage::RemoveComponent(const Entity _entity,
    const ComponentTypeId _typeId)
{
  auto compPtr = this->Component(_entity, _typeId);
  if (nullptr == compPtr)
    return false;

  // if this component is already being ignored, that means it was already
  // removed
  if (compPtr->ignore)
    return false;

  compPtr->ignore = true;
  return true;
}

//////////////////////////////////////////////////
const components::BaseComponent *ComponentStorage::Component(
    const Entity _entity,
    const ComponentTypeId _typeId) const
{
  // make sure the entity exists
  const auto typeMapIter = this->componentTypeIndex.find(_entity);
  if (typeMapIter == this->componentTypeIndex.end())
    return nullptr;

  // make sure the component type exists for the entity
  const auto compIdxIter = typeMapIter->second.find(_typeId);
  if (compIdxIter == typeMapIter->second.end())
    return nullptr;

  // get the pointer to the component
  const auto compVecIter = this->entityComponents.find(_entity);
  if (compVecIter == this->entityComponents.end())
  {
    ignerr << "Entity [" << _entity
      << "] is missing in this->entityComponents, but is in "
      << "this->componentTypeIndex. This should never happen!" << std::endl;
    return nullptr;
  }

  return compVecIter->second.at(compIdxIter->second).get();
}

//////////////////////////////////////////////////
components::BaseComponent *ComponentStorage::Component(const Entity _entity,
    const ComponentTypeId _typeId)
{
  return const_cast<components::BaseComponent *>(
      static_cast<const ComponentStorage &>(*this).Component(_entity, _typeId));
}

//////////////////////////////////////////////////
const components::BaseComponent *ComponentStorage::ValidComponent(
    const Entity _entity, const ComponentTypeId _typeId) const
{
  auto compPtr = this->Component(_entity, _typeId);
  if (nullptr != compPtr && !compPtr->ignore)
    return compPtr;
  return nullptr;
}

//////////////////////////////////////////////////
components::BaseComponent *ComponentStorage::ValidComponent(
    const Entity _entity, const ComponentTypeId _typeId)
{
  auto compPtr = this->Component(_entity, _typeId);
  if (nullptr != compPtr && !compPtr->ignore)
    return compPtr;
  return nullptr;
}
