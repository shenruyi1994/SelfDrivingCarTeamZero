template <typename Arrangement>
void print_arrangement(const Arrangement& arr) {
  CGAL_precondition(arr.is_valid());

  //prints out vertices in arangement
  typename Arrangement::Vertex_const_iterator vit;
  std::cout << arr.number_of_vertices() << " vertices:" << std::endl;
  for (vit = arr.vertices_begin(); vit != arr.vertices_end(); ++vit) {
    std::cout << "(" << vit->point() << ")";
    if (vit->is_isolated()) std::cout << " - Isolated." << std::endl;
    else std::cout << " - degree " << vit->degree() << std::endl;
  }

  //prints out edges in arrangement
  typename Arrangement::Edge_const_iterator eit;
  std::cout << arr.number_of_edges() << " edges:" << std::endl;
  for (eit = arr.edges_begin(); eit != arr.edges_end(); ++eit)
    std::cout << "[" << eit->curve() << "]" << std::endl;

  //prints out faces in arrangement
  //typename Arrangement::face_const_iterator fit;
  //std::cout << arr.number_of_faces() << " faces:" << std::endl;
  //for (fit = arr.faces_begin(); fit != arr.faces_end(); ++fit)
    // print_face<Arrangement>(fit);
}
