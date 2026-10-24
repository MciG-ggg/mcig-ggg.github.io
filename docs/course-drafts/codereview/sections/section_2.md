## 第 2 章 审查清单与沟通心态

code review 要审两样东西：代码里该看的地方，以及你自己带着什么心态坐在那场会里。视频里的团队把审查拆成五个维度，顺序固定：产品需求、可读性、代码规范、对现有逻辑的影响、文档。

清单的用法取决于谁坐在对面。[第 1 章 Code Review 在流程中的位置](<../第 1 章 Code Review 在流程中的位置>)说过参与 review 的三类角色：代码的开发者、技术能力强的同事、QA。QA 在现场不是凑人数，很多 QA 同学本身也写代码。代码里写了判断，QA 会问这些判断是不是全的，有哪些场景会跳过这些判断 [02:26](https://www.bilibili.com/video/BV1Dh4y1J7iG/?t=146)。同一场 review 顺便定下这次改动的范围：你改的是 user control 下面某一个方法，QA 只测这个方法相关的业务逻辑，其余的简单看一眼、不影响流程就放过 [02:36](https://www.bilibili.com/video/BV1Dh4y1J7iG/?t=156)。

### 审查什么：五个维度

![标题卡「到底要review什么？」](images/fig-card-what.png)

**一、产品需求。** 业务逻辑是不是都完全实现了，有没有漏写的代码。发现了就尽量去完善 [02:56](https://www.bilibili.com/video/BV1Dh4y1J7iG/?t=176)。

**二、可读性。** 代码逻辑是否清晰，有没有注释，有没有重复造轮子 [03:07](https://www.bilibili.com/video/BV1Dh4y1J7iG/?t=187)。重复造轮子指什么：别的同事已经实现了你想要的这个功能，你不知道，于是又写了一遍。撞上了直接复用已有的 [03:10](https://www.bilibili.com/video/BV1Dh4y1J7iG/?t=190)。

**三、代码规范。** 进来的参数有没有做校验，判空，空集合；查询数据库有没有加 `LIMIT`，别造成一个非常大的查询把服务给撑爆；权限和数据安全方面有没有做相应的 check [03:23](https://www.bilibili.com/video/BV1Dh4y1J7iG/?t=203)。

`LIMIT` 这一条最容易被漏。有没有它，决定了这次查询是走索引还是扫全表：

```sql
SELECT id, order_no, amount, created_at
FROM orders
WHERE user_id = :user_id
ORDER BY created_at DESC
LIMIT 200;
```

**四、对现有逻辑的影响。** 新功能依赖一个三方 interface 时，要问三方系统挂了或者 interface 报错，会不会影响到自己系统的主流程——用户下不了单、没有办法支付，这些就是线上故障。这段代码有没有加 try-catch，有没有做一些降级策略。这一步每次做 code review 都要花很长时间，也非常重要 [03:42](https://www.bilibili.com/video/BV1Dh4y1J7iG/?t=222)。

降级一般长这样，三方不可用时退回本地缓存，主流程继续走完：

```python
try:
    return payment_gateway.query(order_no)
except GatewayTimeout:
    logger.warning("payment gateway timeout, fallback to cache: %s", order_no)
    return cache.get_cached(order_no)
```

**五、文档。** 本次涉及库表改动、interface 路径变更这些，一定要维护一份非常清晰的文档。上线之前的发布准备，全靠这份文档 [04:06](https://www.bilibili.com/video/BV1Dh4y1J7iG/?t=246)。


### 心态：审代码，别审人

![标题卡「心态很重要！」](images/fig-card-mindset.png)

> 我第一次参加 code review 的时候感觉自己在一件一件脱光衣服给别人看 [04:26](https://www.bilibili.com/video/BV1Dh4y1J7iG/?t=266)

难受的往往不是清单本身，是对话。高阶的同事前期没有参加过需求评审，他甚至不知道你这期要开发的是什么样的需求、要做什么样的迭代，所以你还得去讲需求背景、解释为什么这么做。心里非常清楚的一件事情一遍一遍地解释给别人，非常容易失去耐心，情绪管理也可能失控 [04:37](https://www.bilibili.com/video/BV1Dh4y1J7iG/?t=277)。

换一个姿势。视频里的做法是毫无保留：

> 我通常会毫无保留地去告诉每一个人我为什么要这么做，然后当时写代码的时候我的一个想法是怎样的 [05:08](https://www.bilibili.com/video/BV1Dh4y1J7iG/?t=308)

对方一旦全面了解了这件事，给出来的建议都是非常值得采纳的，甚至还会给出更好的解决方案。坐在对面的是架构师、开发组长、CTO 这个级别的人，他们身上有可取之处。视频里每次 code review 都抱着一种学习的心态 [05:26](https://www.bilibili.com/video/BV1Dh4y1J7iG/?t=326)。

反过来的状态是「你看不上我、我看不上你」，这样的 code review 就没有啥意义了。有些程序员直接 code review 完就离职，觉得公司太磨叽，对自己不信任，连细枝末节咋写的、咋实现的都要管 [05:47](https://www.bilibili.com/video/BV1Dh4y1J7iG/?t=347)。

该坚持的时候也要坚持。一件事情你怎么样去做，没有绝对的好，也没有绝对的坏。表达观点、和别人沟通的时候要有耐心，要去想怎么去说服别人 [06:07](https://www.bilibili.com/video/BV1Dh4y1J7iG/?t=367)。

说服别人本身是一种非常重要的技能，沟通的技能同样重要。这一段收尾时说的是：

> 总的来说就是为人谦虚一点然后多听取别人的建议 [06:25](https://www.bilibili.com/video/BV1Dh4y1J7iG/?t=385)
