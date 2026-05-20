#ifndef STARTER_MODEL_H
#define STARTER_MODEL_H

#include <stdint.h>

#define STARTER_MODEL_FEATURE_COUNT 11
#define STARTER_MODEL_CLASS_COUNT 6
#define STARTER_MODEL_TREE_COUNT 15

typedef struct {
    int16_t left_index;
    int16_t right_index;
    int8_t feature_index;
    uint8_t predicted_class;
    float threshold;
} starter_model_node_t;

static const float starter_model_feature_mean[STARTER_MODEL_FEATURE_COUNT] = {0.08971514f, 0.04902724f, 0.78156026f, 0.04690346f, 13.49702595f, 0.39777756f, 0.24731381f, 0.02392585f, 0.24418881f, 0.74975574f, 0.24731381f};
static const float starter_model_feature_std[STARTER_MODEL_FEATURE_COUNT] = {0.08747651f, 0.06997486f, 0.29654548f, 0.04302364f, 7.08896110f, 0.28042717f, 0.26981583f, 0.06973812f, 0.18211722f, 0.27508731f, 0.26981583f};
static const char *const starter_model_label_names[STARTER_MODEL_CLASS_COUNT] = {"CLAP", "KNOCK", "GLASS", "COUGH", "LAUGH", "TICK"};

static const starter_model_node_t starter_model_tree_0[13] = {
    {1, 10, 8, 2, 1.44903251f},
    {2, 7, 3, 0, 0.51133157f},
    {3, 4, 5, 1, -0.90277165f},
    {-1, -1, -1, 4, 0.00000000f},
    {5, 6, 10, 1, -0.77614995f},
    {-1, -1, -1, 5, 0.00000000f},
    {-1, -1, -1, 1, 0.00000000f},
    {8, 9, 10, 0, -0.38067622f},
    {-1, -1, -1, 2, 0.00000000f},
    {-1, -1, -1, 0, 0.00000000f},
    {11, 12, 1, 2, -0.18997127f},
    {-1, -1, -1, 2, 0.00000000f},
    {-1, -1, -1, 4, 0.00000000f}
};
static const starter_model_node_t starter_model_tree_1[11] = {
    {1, 2, 0, 3, -0.73338566f},
    {-1, -1, -1, 3, 0.00000000f},
    {3, 10, 4, 0, 0.96706609f},
    {4, 7, 3, 0, 0.60693942f},
    {5, 6, 7, 5, -0.27615982f},
    {-1, -1, -1, 5, 0.00000000f},
    {-1, -1, -1, 4, 0.00000000f},
    {8, 9, 8, 0, 1.27282685f},
    {-1, -1, -1, 2, 0.00000000f},
    {-1, -1, -1, 0, 0.00000000f},
    {-1, -1, -1, 1, 0.00000000f}
};
static const starter_model_node_t starter_model_tree_2[13] = {
    {1, 8, 4, 2, 0.64983917f},
    {2, 3, 0, 2, -0.34483256f},
    {-1, -1, -1, 5, 0.00000000f},
    {4, 7, 1, 2, 2.24778619f},
    {5, 6, 4, 2, -0.14314005f},
    {-1, -1, -1, 2, 0.00000000f},
    {-1, -1, -1, 2, 0.00000000f},
    {-1, -1, -1, 0, 0.00000000f},
    {9, 10, 4, 4, 0.95255838f},
    {-1, -1, -1, 4, 0.00000000f},
    {11, 12, 5, 1, -0.30678742f},
    {-1, -1, -1, 1, 0.00000000f},
    {-1, -1, -1, 3, 0.00000000f}
};
static const starter_model_node_t starter_model_tree_3[15] = {
    {1, 8, 2, 1, 0.22087976f},
    {2, 3, 4, 3, -0.08367559f},
    {-1, -1, -1, 5, 0.00000000f},
    {4, 7, 5, 3, -0.69343985f},
    {5, 6, 0, 4, -0.56093044f},
    {-1, -1, -1, 4, 0.00000000f},
    {-1, -1, -1, 3, 0.00000000f},
    {-1, -1, -1, 3, 0.00000000f},
    {9, 14, 0, 1, 1.11701184f},
    {10, 11, 7, 1, -0.33604476f},
    {-1, -1, -1, 1, 0.00000000f},
    {12, 13, 9, 0, -1.43291382f},
    {-1, -1, -1, 0, 0.00000000f},
    {-1, -1, -1, 2, 0.00000000f},
    {-1, -1, -1, 0, 0.00000000f}
};
static const starter_model_node_t starter_model_tree_4[17] = {
    {1, 8, 2, 3, 0.22087976f},
    {2, 3, 6, 3, -0.84155475f},
    {-1, -1, -1, 3, 0.00000000f},
    {4, 5, 9, 3, 0.66709786f},
    {-1, -1, -1, 3, 0.00000000f},
    {6, 7, 9, 5, 0.80949428f},
    {-1, -1, -1, 5, 0.00000000f},
    {-1, -1, -1, 4, 0.00000000f},
    {9, 12, 7, 2, -0.22038236f},
    {10, 11, 0, 2, -0.47596747f},
    {-1, -1, -1, 1, 0.00000000f},
    {-1, -1, -1, 2, 0.00000000f},
    {13, 14, 7, 4, -0.02233840f},
    {-1, -1, -1, 4, 0.00000000f},
    {15, 16, 4, 0, -0.14491779f},
    {-1, -1, -1, 0, 0.00000000f},
    {-1, -1, -1, 2, 0.00000000f}
};
static const starter_model_node_t starter_model_tree_5[15] = {
    {1, 6, 1, 3, -0.54636570f},
    {2, 5, 4, 3, 0.70737292f},
    {3, 4, 3, 5, -0.65272296f},
    {-1, -1, -1, 4, 0.00000000f},
    {-1, -1, -1, 5, 0.00000000f},
    {-1, -1, -1, 3, 0.00000000f},
    {7, 8, 4, 2, -1.16261174f},
    {-1, -1, -1, 0, 0.00000000f},
    {9, 12, 7, 2, -0.19997214f},
    {10, 11, 10, 2, -0.28023305f},
    {-1, -1, -1, 5, 0.00000000f},
    {-1, -1, -1, 2, 0.00000000f},
    {13, 14, 2, 4, 0.62989667f},
    {-1, -1, -1, 4, 0.00000000f},
    {-1, -1, -1, 0, 0.00000000f}
};
static const starter_model_node_t starter_model_tree_6[17] = {
    {1, 8, 8, 1, -0.46799868f},
    {2, 3, 7, 5, -0.34037814f},
    {-1, -1, -1, 3, 0.00000000f},
    {4, 5, 0, 5, -0.43934760f},
    {-1, -1, -1, 5, 0.00000000f},
    {6, 7, 4, 1, -0.76131905f},
    {-1, -1, -1, 4, 0.00000000f},
    {-1, -1, -1, 1, 0.00000000f},
    {9, 12, 7, 1, -0.29657005f},
    {10, 11, 3, 1, -0.06442374f},
    {-1, -1, -1, 1, 0.00000000f},
    {-1, -1, -1, 2, 0.00000000f},
    {13, 14, 6, 4, 0.76385855f},
    {-1, -1, -1, 0, 0.00000000f},
    {15, 16, 1, 4, -0.49050084f},
    {-1, -1, -1, 2, 0.00000000f},
    {-1, -1, -1, 4, 0.00000000f}
};
static const starter_model_node_t starter_model_tree_7[15] = {
    {1, 8, 3, 1, 0.15398802f},
    {2, 3, 6, 1, -0.81410115f},
    {-1, -1, -1, 4, 0.00000000f},
    {4, 5, 2, 1, -0.46897930f},
    {-1, -1, -1, 5, 0.00000000f},
    {6, 7, 5, 1, 0.26464724f},
    {-1, -1, -1, 1, 0.00000000f},
    {-1, -1, -1, 4, 0.00000000f},
    {9, 14, 4, 0, 0.49332438f},
    {10, 13, 10, 0, 1.26150913f},
    {11, 12, 7, 0, -0.16144301f},
    {-1, -1, -1, 2, 0.00000000f},
    {-1, -1, -1, 0, 0.00000000f},
    {-1, -1, -1, 2, 0.00000000f},
    {-1, -1, -1, 3, 0.00000000f}
};
static const starter_model_node_t starter_model_tree_8[15] = {
    {1, 8, 0, 3, -0.30125521f},
    {2, 3, 8, 3, -0.56311664f},
    {-1, -1, -1, 3, 0.00000000f},
    {4, 5, 0, 5, -0.70912965f},
    {-1, -1, -1, 5, 0.00000000f},
    {6, 7, 6, 1, -0.73237580f},
    {-1, -1, -1, 3, 0.00000000f},
    {-1, -1, -1, 1, 0.00000000f},
    {9, 10, 7, 2, -0.33205041f},
    {-1, -1, -1, 1, 0.00000000f},
    {11, 14, 5, 2, 1.22991099f},
    {12, 13, 1, 2, 2.24778619f},
    {-1, -1, -1, 2, 0.00000000f},
    {-1, -1, -1, 0, 0.00000000f},
    {-1, -1, -1, 4, 0.00000000f}
};
static const starter_model_node_t starter_model_tree_9[9] = {
    {1, 8, 4, 3, 0.74041066f},
    {2, 7, 4, 4, 0.64983917f},
    {3, 4, 0, 5, -0.43934760f},
    {-1, -1, -1, 5, 0.00000000f},
    {5, 6, 8, 2, 0.52253937f},
    {-1, -1, -1, 0, 0.00000000f},
    {-1, -1, -1, 2, 0.00000000f},
    {-1, -1, -1, 4, 0.00000000f},
    {-1, -1, -1, 3, 0.00000000f}
};
static const starter_model_node_t starter_model_tree_10[13] = {
    {1, 6, 3, 1, -0.85258237f},
    {2, 3, 5, 1, -0.24285713f},
    {-1, -1, -1, 1, 0.00000000f},
    {4, 5, 6, 1, 0.40129429f},
    {-1, -1, -1, 3, 0.00000000f},
    {-1, -1, -1, 1, 0.00000000f},
    {7, 12, 6, 2, 0.91134275f},
    {8, 9, 2, 3, -1.16274894f},
    {-1, -1, -1, 5, 0.00000000f},
    {10, 11, 2, 3, 0.47017992f},
    {-1, -1, -1, 3, 0.00000000f},
    {-1, -1, -1, 0, 0.00000000f},
    {-1, -1, -1, 2, 0.00000000f}
};
static const starter_model_node_t starter_model_tree_11[11] = {
    {1, 8, 3, 1, 0.51133157f},
    {2, 3, 9, 1, -1.96331517f},
    {-1, -1, -1, 2, 0.00000000f},
    {4, 5, 0, 1, -0.73299669f},
    {-1, -1, -1, 3, 0.00000000f},
    {6, 7, 9, 1, 0.83591822f},
    {-1, -1, -1, 1, 0.00000000f},
    {-1, -1, -1, 3, 0.00000000f},
    {9, 10, 1, 0, -0.02072470f},
    {-1, -1, -1, 2, 0.00000000f},
    {-1, -1, -1, 0, 0.00000000f}
};
static const starter_model_node_t starter_model_tree_12[17] = {
    {1, 8, 2, 3, 0.24537241f},
    {2, 7, 8, 3, -0.56311664f},
    {3, 6, 7, 3, -0.28162534f},
    {4, 5, 0, 3, -0.84767084f},
    {-1, -1, -1, 3, 0.00000000f},
    {-1, -1, -1, 4, 0.00000000f},
    {-1, -1, -1, 3, 0.00000000f},
    {-1, -1, -1, 5, 0.00000000f},
    {9, 12, 3, 1, -0.44669701f},
    {10, 11, 10, 1, 0.60398763f},
    {-1, -1, -1, 1, 0.00000000f},
    {-1, -1, -1, 4, 0.00000000f},
    {13, 14, 9, 2, -0.16670672f},
    {-1, -1, -1, 2, 0.00000000f},
    {15, 16, 8, 4, -0.87634331f},
    {-1, -1, -1, 4, 0.00000000f},
    {-1, -1, -1, 0, 0.00000000f}
};
static const starter_model_node_t starter_model_tree_13[15] = {
    {1, 6, 2, 1, 0.22087976f},
    {2, 5, 5, 3, -0.59037950f},
    {3, 4, 10, 3, -0.81238998f},
    {-1, -1, -1, 3, 0.00000000f},
    {-1, -1, -1, 5, 0.00000000f},
    {-1, -1, -1, 3, 0.00000000f},
    {7, 8, 0, 1, -0.47689683f},
    {-1, -1, -1, 1, 0.00000000f},
    {9, 12, 1, 2, -0.02072470f},
    {10, 11, 0, 2, -0.31062552f},
    {-1, -1, -1, 4, 0.00000000f},
    {-1, -1, -1, 2, 0.00000000f},
    {13, 14, 8, 0, -0.87634331f},
    {-1, -1, -1, 4, 0.00000000f},
    {-1, -1, -1, 0, 0.00000000f}
};
static const starter_model_node_t starter_model_tree_14[19] = {
    {1, 10, 5, 4, 0.05376268f},
    {2, 7, 4, 1, -0.07558774f},
    {3, 6, 10, 5, -0.77614995f},
    {4, 5, 9, 5, 0.80915337f},
    {-1, -1, -1, 5, 0.00000000f},
    {-1, -1, -1, 2, 0.00000000f},
    {-1, -1, -1, 1, 0.00000000f},
    {8, 9, 4, 3, 0.96706609f},
    {-1, -1, -1, 3, 0.00000000f},
    {-1, -1, -1, 1, 0.00000000f},
    {11, 14, 10, 0, 0.76385855f},
    {12, 13, 5, 0, 0.50227629f},
    {-1, -1, -1, 2, 0.00000000f},
    {-1, -1, -1, 0, 0.00000000f},
    {15, 16, 2, 4, 0.69776600f},
    {-1, -1, -1, 4, 0.00000000f},
    {17, 18, 0, 2, -0.10521108f},
    {-1, -1, -1, 2, 0.00000000f},
    {-1, -1, -1, 0, 0.00000000f}
};

static const uint16_t starter_model_tree_sizes[STARTER_MODEL_TREE_COUNT] = {13, 11, 13, 15, 17, 15, 17, 15, 15, 9, 13, 11, 17, 15, 19};
static const starter_model_node_t *const starter_model_trees[STARTER_MODEL_TREE_COUNT] = {starter_model_tree_0, starter_model_tree_1, starter_model_tree_2, starter_model_tree_3, starter_model_tree_4, starter_model_tree_5, starter_model_tree_6, starter_model_tree_7, starter_model_tree_8, starter_model_tree_9, starter_model_tree_10, starter_model_tree_11, starter_model_tree_12, starter_model_tree_13, starter_model_tree_14};

static int starter_model_tree_predict(const starter_model_node_t *tree, uint16_t node_count, const float *x)
{
    int16_t index = 0;

    while ((index >= 0) && (index < (int16_t)node_count)) {
        const starter_model_node_t *node = &tree[index];
        if (node->feature_index < 0) {
            return (int)node->predicted_class;
        }
        if (x[node->feature_index] <= node->threshold) {
            index = node->left_index;
        } else {
            index = node->right_index;
        }
    }

    return 0;
}

static int starter_model_predict_class(const float *features)
{
    float x[STARTER_MODEL_FEATURE_COUNT];
    uint8_t votes[STARTER_MODEL_CLASS_COUNT] = {0};
    int best_class = 0;

    for (int i = 0; i < STARTER_MODEL_FEATURE_COUNT; ++i) {
        x[i] = (features[i] - starter_model_feature_mean[i]) / starter_model_feature_std[i];
    }

    for (int tree_index = 0; tree_index < STARTER_MODEL_TREE_COUNT; ++tree_index) {
        int predicted_class = starter_model_tree_predict(
            starter_model_trees[tree_index],
            starter_model_tree_sizes[tree_index],
            x
        );
        if ((predicted_class >= 0) && (predicted_class < STARTER_MODEL_CLASS_COUNT)) {
            votes[predicted_class]++;
        }
    }

    for (int class_index = 1; class_index < STARTER_MODEL_CLASS_COUNT; ++class_index) {
        if (votes[class_index] > votes[best_class]) {
            best_class = class_index;
        }
    }

    return best_class;
}

static const char *starter_model_class_name(int class_id)
{
    if ((class_id < 0) || (class_id >= STARTER_MODEL_CLASS_COUNT)) {
        return "UNK";
    }
    return starter_model_label_names[class_id];
}

#endif
