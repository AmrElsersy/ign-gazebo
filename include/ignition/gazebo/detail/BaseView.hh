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
#ifndef IGNITION_GAZEBO_DETAIL_BASEVIEW_HH_
#define IGNITION_GAZEBO_DETAIL_BASEVIEW_HH_

#include <cstddef>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "ignition/gazebo/Entity.hh"
#include "ignition/gazebo/Types.hh"
#include "ignition/gazebo/config.hh"

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace detail
{
/// \brief A key into the map of views
using ComponentTypeKey = std::vector<ComponentTypeId>;

/// \brief Hash functor for ComponentTypeKey
struct ComponentTypeHasher
{
  std::size_t operator()(const std::vector<ComponentTypeId> &_vec) const
  {
    auto hash = _vec.size();
    for (const auto &i : _vec)
      hash ^= i + 0x9e3779b9 + (hash << 6) + (hash >> 2);
    return hash;
  }
};

/// \brief A view is a cache to entities, and their components, that
/// match a set of component types. A cache is used because systems will
/// frequently, potentially every iteration, query the
/// EntityComponentManager for sets of entities that match a set of
/// component types. Rather than look up the entities every time, we can
/// use a cache to improve performance.
///
/// Note that symbols for this class are visible because methods from this class
/// are used in templated Ignition::Gazebo::EntityComponentManager methods.
/// However, users should not use this class (or anything else in namespace
/// ignition::gazebo::detail) directly.
class IGNITION_GAZEBO_VISIBLE BaseView
{
  /// \brief See if an entity is a part of the view
  /// \param[in] _entity The entity
  /// \return true if _entity is a part of the view, false otherwise
  public: bool HasEntity(const Entity _entity) const;

  /// \brief See if an entity is marked as an entity to be added to the view
  /// \param[in] _entity The entity
  /// \return true if _entity is to be added to the view, false otherwise
  public: bool HasEntityToAdd(const Entity _entity) const;

  /// \brief See if the view has cached the component data for an entity
  /// \param[in] _entity The entity
  /// \return true if _entity has component data cached in the view, false
  /// otherwise
  public: virtual bool HasCachedComponentData(const Entity _entity) const = 0;

  /// \brief Save an entity as one to be added to the view, if the entity isn't
  /// already a part of the view. This entity's component data should be added
  /// to the view the next time the view is being used.
  /// \param[in] _entity The entity to be added
  /// \param[in] _new Whether to add the entity to the list of new entities.
  /// The new here is to indicate whether the entity is new to the entity
  /// component manager. An existing entity can be added when creating a new
  /// view or when rebuilding the view.
  /// \return True if _entity isn't already a part of the view and was marked as
  /// an entity to be added. False otherwise
  /// \sa HasEntity, HasEntityToAdd
  public: bool MarkEntityToAdd(const Entity _entity, const bool _new = false);

  /// \brief See if the view requires a particular component type
  /// \param[in] _typeId The component type
  /// \return true if the view requires components of type _typeId, false
  /// otherwise
  public: bool RequiresComponent(const ComponentTypeId _typeId) const;

  /// \brief Update the internal data in the view because a component has been
  /// added to an entity. It is assumed that the entity is already associated
  /// with the view, and that the added component type is required by the view.
  /// \param[in] _entity The entity
  /// \param[in] _typeId The type of component that was added to _entity
  /// \return true if the notification related to the component addition of type
  /// _typeId occurred for _entity, false otherwise
  /// \sa HasCachedComponentData, RequiresComponent
  public: virtual bool NotifyComponentAddition(const Entity _entity,
              const ComponentTypeId _typeId) = 0;

  /// \brief Update the internal data in the view because a component has been
  /// removed to an entity. It is assumed that the entity is already associated
  /// with the view, and that the removed component type is required by the view
  /// \param[in] _entity The entity
  /// \param[in] _typeId The type of component that was removed from _entity
  /// \return true if the notification related to the component removal of type
  /// _typeId occurred for _entity, false otherwise
  /// \sa HasCachedComponentData, RequiresComponent
  public: virtual bool NotifyComponentRemoval(const Entity _entity,
              const ComponentTypeId _typeId) = 0;

  /// \brief Remove an entity from the view.
  /// \param[in] _entity The entity to remove.
  /// \return True if the entity was removed, false if the entity did not
  /// exist in the view.
  public: virtual bool RemoveEntity(const Entity _entity) = 0;

  /// \brief Add the entity to the list of entities to be removed
  /// \param[in] _entity The entity to add.
  /// \return True if the entity was added to the list, false if the entity
  /// was not associated with the view.
  // TODO make this pure virtual since we may need to remove it from invalidData
  // and missingCompTracker? I'll need to look into how removal occurs
  // * actually, after looking at this further, I believe that this method
  // (and the toRemoveEntities data member) are not needed. I'll re-visit this later *
  public: bool AddEntityToRemoved(const Entity _entity);

  /// \brief Clear the list of new entities
  public: void ClearNewEntities();

  /// \brief Get the set of component types that this view requires.
  /// \return The set of component types.
  public: const std::unordered_set<ComponentTypeId> &ComponentTypes() const;

  /// \brief Clear all data from the view and reset it to its original, empty
  /// state.
  public: virtual void Reset() = 0;

  /// \brief Destructor
  public: virtual ~BaseView()
  {
  }

  // TODO make this private/protected and have public API methods for this container?
  // TODO make this a std::unordered_set for better performance. We need to make
  // sure nothing else depends on the ordered preserved by std::set first
  /// \brief All the entities that belong to this view.
  public: std::set<Entity> entities;

  // TODO make this private/protected and have public API methods for this container?
  // TODO make this a std::unordered_set for better performance. We need to make
  // sure nothing else depends on the ordered preserved by std::set first
  /// \brief List of newly created entities
  public: std::set<Entity> newEntities;

  // TODO make this private/protected and have public API methods for this container?
  // TODO make this a std::unordered_set for better performance. We need to make
  // sure nothing else depends on the ordered preserved by std::set first
  /// \brief List of entities about to be removed
  public: std::set<Entity> toRemoveEntities;

  // TODO make this private/protected and have public API methods for this container?
  // (remember to clear this after adding the entities to the view)
  /// \brief List of entities to be added to the view. The value of the map
  /// indicates whether the entity is new to the entity component manager or not
  public: std::unordered_map<Entity, bool> toAddEntities;

  /// \brief The component types in the view
  protected: std::unordered_set<ComponentTypeId> componentTypes;
};
} // namespace detail
} // namespace IGNITION_GAZEBO_VERSION_NAMESPACE
} // namespace gazebo
} // namespace ignition
#endif
