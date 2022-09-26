class DataSource(object):

    def __init__(self, label, interval):
        self.label = label
        self.status = 0
        self.interval = interval
        self.round = int(len(label) / interval)

    def get_batch_data(self):
        res = self.label[int(self.status*self.interval): int((self.status+1)*self.interval)]
        self.status = (self.status + 1) % self.round
        return res