def modify_field_value_2_0(input_filename, output_filename,field_idx):
    try:
        with open(input_filename, 'r') as infile, open(output_filename, 'w') as outfile:
            for line in infile:
                tokens = line.split()
                if len(tokens) >= 3:
                    tokens[field_idx] = '0'
                outfile.write(' '.join(tokens) + '\n')
    except IOError as e:
        print(f"An error occurred while handling the file: {e}")

def main():
    input_filename = 'E:/my-github-repos/149_pcd/bunny.txt'  # 替换为你的输入文件名
    output_filename1 = 'E:/my-github-repos/149_pcd/bunny_yz.txt'  # 替换为你的输出文件名
    output_filename2 = 'E:/my-github-repos/149_pcd/bunny_xz.txt'  # 替换为你的输出文件名
    output_filename3 = 'E:/my-github-repos/149_pcd/bunny_xy.txt'  # 替换为你的输出文件名

    modify_field_value_2_0(input_filename, output_filename1,0)
    modify_field_value_2_0(input_filename, output_filename2,1)
    modify_field_value_2_0(input_filename, output_filename3,2)

if __name__ == '__main__':
    main()
