import numpy as np

class Bins():

    def __init__(self):
        self.nbins = 10
        self.bins_range = (0,256)


class Color_List():

    def __init__(self, list_1, list_2, list_3):
        self.list_1 = list_1
        self.list_2 = list_2
        self.list_3 = list_3


class Histogram_Dictionary():

        def __init__(self):
            self.hist_1 = None
            self.hist_2 = None
            self.hist_3 = None
            self.hist_features = None
            self.norm_features = None


class Histogram(Bins,Color_List, Histogram_Dictionary):

    def __init__(self, list_1, list_2, list_3):
        Bins.__init__(self)
        Color_List.__init__(self, list_1, list_2, list_3)
        Histogram_Dictionary.__init__(self)

    def generate_normalised_concatenated_histogram(self):

        self.generate_3_histograms_from_class_instance()
        self.concatenate_hist_1_2_3()
        self.normalise_histogram_features()

        return self.norm_features


    def generate_3_histograms_from_class_instance(self):

        self.hist_1 = np.histogram(self.list_1,
                                   bins=self.nbins,
                                   range=self.bins_range)
        self.hist_2 = np.histogram(self.list_2,
                                   bins=self.nbins,
                                   range=self.bins_range)
        self.hist_3 = np.histogram(self.list_3,
                                   bins=self.nbins,
                                   range=self.bins_range)
        return


    def concatenate_hist_1_2_3(self):

        self.hist_features =np.concatenate((self.hist_1[0],
                                   self.hist_2[0],
                                   self.hist_3[0])).astype(np.float64)
        return


    def normalise_histogram_features(self):

        self.norm_features = self.hist_features / np.sum(self.hist_features)

        return

