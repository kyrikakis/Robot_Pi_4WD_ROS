"""What the Hailo face recognition's save_face answered (no ROS, so it can be tested).

Its node does not use the codes in SaveGalleryItem.srv's comment: it answers 1
for all three "already known" cases and tells them apart only in the message
(hailo_tappas_ros2 face_recognition_node.py, add_face_callback):
  0 "Person added"
  1 "Similar embedding found with the same id ..."      this face, this name
  1 "Similar embedding found with different id ..."     this face, another name
  1 "The id exists with distant embedding(s) ..."       this name, another-looking face
  2 multiple faces, 3 no face, 4 error

append=true adds the face to an existing name, but with a face close to someone
else's it renames that person: use it only after 'distant' (embeddings_gallery.py
append_new_item).
"""


def save_outcome(result, message):
    """'added', 'same', 'other_name', 'distant', 'many', 'none' or 'error'."""
    text = (message or '').lower()
    if result == 0:
        return 'added'
    if result == 1:
        if 'distant' in text:
            return 'distant'
        if 'different id' in text:
            return 'other_name'
        return 'same'
    return {2: 'many', 3: 'none'}.get(result, 'error')
