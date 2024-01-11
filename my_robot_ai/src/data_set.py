import random

def build_data_set():
    # Output data
    sizes = ["small", "medium", "large"]
    colors = ["yellow", "red", "blue", "green", "purple"]

    # Augment data
    suface_syn = ["platform", "surface", "area", "base", "block", "ground", "location"]
    objs = ["box", "cube", "object", "thing", "item"]

    text_data = []
    size_data = []
    color_data = []

    aug_text_data = []

    for size in sizes:
        for color in colors:
            for syn in suface_syn:
                for obj in objs:
                    aug_text_data.extend([
                        [f"place the {size} {obj} on the {color} {syn}", size, color],
                        [f"place {size} {obj} on {color} the {syn}", size, color],
                        [f"please place the {size} {obj} on the {color} {syn}", size, color],
                        [f"can you place the {size} {obj} on the {color} {syn}", size, color],

                        [f"put the {size} {obj} on the {color} {syn}",size, color],
                        [f"put {size} {obj} on {color} the {syn}",size, color],
                        [f"please put the {size} {obj} on the {color} {syn}",size, color],
                        [f"can you put the {size} {obj} on the {color} {syn}",size, color],

                        [f"move the {size} {obj} to the {color} {syn}",size, color],
                        [f"move {size} {obj} to {color} the {syn}",size, color],
                        [f"please move the {size} {obj} to the {color} {syn}",size, color],
                        [f"please move {size} {obj} to the {color} {syn}",size, color],
                        [f"can you move the {size} {obj} to the {color} {syn}",size, color],
                        [f"can you move {size} {obj} to the {color} {syn}",size, color],
                        [f"can you move the {size} {obj} from the {random.choice(colors)} {syn} to the {color} {syn}",size, color],
                        [f"can you move {size} {obj} from the {random.choice(colors)} {syn} to the {color} {syn}",size, color],
                        [f"move the {size} {obj} from the {random.choice(colors)} {syn} to the {color} {syn}",size, color],

                        [f"pick the {size} {obj} and place on the {color} {syn}",size, color],
                        [f"pick {size} {obj} and place on {color} {syn}",size, color],
                        [f"please pick the {size} {obj} and place on the {color} {syn}",size, color],
                        [f"can you pick the {size} {obj} and place on the {color} {syn}",size, color],
                        [f"pick up the {size} {obj} from the {random.choice(colors)} {syn} to the {color} {syn}",size, color],
                        [f"pick the {size} {obj} from the {random.choice(colors)} {syn} and move to the {color} {syn}",size, color],
                        [f"pick the {size} {obj} from the {random.choice(colors)} {syn} and place the {color} {syn}",size, color],
                        [f"pick the {size} {obj} from the {random.choice(colors)} {syn} and put on the {color} {syn}",size, color],
                        [f"pick up the {size} {obj} from the {random.choice(colors)} {syn} and move to the {color} {syn}",size, color],
                    ])
    random.shuffle(aug_text_data)

    for d in aug_text_data:
        text_data.append(d[0])
        size_data.append(d[1])
        color_data.append(d[2])

    data = {
        'text': text_data,
        'size': size_data,
        'color': color_data,
    }

    return data
