import octomap
def main(args=None):
    file_path="/home/jimmy/Downloads/newmap.bt" 



    occupancy_map = octomap.OcTree(0.05)  #self.process_octomap(msg)# 5 cm resolution
  
        #file_path = file_path.decode("utf-8")
    if isinstance(file_path, bytes):  
        file_path = file_path.decode("utf-8")  # ? Convert bytes to string

        # ? Load the Octomap from the file
    if occupancy_map.readBinary(file_path.encode()):  
        print("? Octomap loaded successfully!")
        print(f"Number of occupied nodes: {sum(1 for _ in occupancy_map.begin_leafs())}")
    else:
        print("? Error: Could not load Octomap from file!")


if __name__ == "__main__":
    main()